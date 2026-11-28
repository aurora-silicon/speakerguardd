# SPDX-License-Identifier: MIT
import errno, fcntl, io, logging, os, os.path, secrets, tarfile
from hashlib import sha256
from . import cpio

UBOOT_FILES = set([
    "asmedia/asm2214a-apple.bin"
])

class FWFile(object):
    def __init__(self, name, data, sha=None):
        # sha: data's known SHA-256 (an alias of a file already read).
        self.name = name
        self.data = data
        self.sha = sha or sha256(data).hexdigest()

    def __repr__(self):
        return f"FWFile({self.name!r}, <{self.sha[:16]}>)"

    def __eq__(self, other):
        if other is None:
            return False
        return self.sha == other.sha

    def __hash__(self):
        return hash(self.sha)

class FWPackageBusy(RuntimeError):
    """Another run is writing a package into the same directory."""

class FWPackage(object):
    # The package is built under temporary names beside the final ones and
    # renamed into place by close(), so a run that fails (or never reaches
    # close()) leaves the previous package as it was.
    #
    # close() renames the files one at a time, manifest.txt last. When a
    # rename fails, the files already renamed are put back from the copies
    # kept of the previous ones, so the directory holds the old package or
    # the new one, not a mix. A crash or power loss during close() can leave
    # a mix, or a missing file on filesystems where the backup is renamed
    # aside. A new manifest is installed only after the package files.
    #
    # The directory is locked (flock) from the start to the end of the
    # package: a second run on the same directory fails with FWPackageBusy
    # instead of deleting this one's temporary files.
    def __init__(self, path):
        self.closed = True
        self.path = path
        self.tar_path = os.path.join(path, "firmware.tar")
        self.cpio_path = os.path.join(path, "firmware.cpio")
        self.manifest_path = os.path.join(path, "manifest.txt")
        self._staged = {}
        self._uboot = []
        self._lock = None
        try:
            self._take_lock()
            self.tarfile = tarfile.open(self._stage(self.tar_path), mode="w")
            self.cpiofile = cpio.CPIO(self._stage(self.cpio_path))
        except BaseException:
            self.abort()
            raise
        self.closed = False
        self.hashes = {}
        self.manifest = []

    def _take_lock(self):
        fd = os.open(self.path or ".", os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            os.close(fd)
            raise FWPackageBusy(f"Another firmware update is writing to {self.path}; "
                                "try again when it is done") from None
        except BaseException:
            os.close(fd)
            raise
        self._lock = fd

    def _release_lock(self):
        if self._lock is not None:
            os.close(self._lock) # drops the flock
            self._lock = None

    @staticmethod
    def _remove_stale(dirname, base):
        # A run that was killed leaves its temporary files: drop them, so
        # they do not add up on a small partition. The lock is held, so no
        # other run is using them.
        prefix = f".{base}."
        for name in os.listdir(dirname or "."):
            if name.startswith(prefix) and name.endswith((".tmp", ".old")):
                try:
                    os.unlink(os.path.join(dirname, name))
                except OSError:
                    pass

    @staticmethod
    def _create(dirname, base, suffix):
        """Create a new empty file .base.XXXXXXXX.suffix in dirname. Its
        mode is the one a plain open() gives (0666 less the umask)."""
        flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0)
        while True:
            path = os.path.join(dirname, f".{base}.{secrets.token_hex(4)}.{suffix}")
            try:
                return path, os.open(path, flags, 0o666)
            except FileExistsError:
                continue

    def _stage(self, final):
        """A new empty file beside final, to be renamed to it on close()."""
        dirname, base = os.path.split(final)
        self._remove_stale(dirname, base)
        tmp, fd = self._create(dirname or ".", base, "tmp")
        os.close(fd)
        self._staged[final] = tmp
        return tmp

    def _write_staged(self, final, data):
        with open(self._stage(final), "wb") as fd:
            fd.write(data)

    @staticmethod
    def _fsync(path, directory=False):
        fd = os.open(path, os.O_RDONLY | (getattr(os, "O_DIRECTORY", 0) if directory else 0))
        try:
            os.fsync(fd)
        except OSError as e:
            # Some filesystems cannot sync a directory; the renames are
            # then as durable as they make them.
            if not directory or e.errno not in (errno.EINVAL, errno.ENOTSUP, errno.EBADF):
                raise
        finally:
            os.close(fd)

    def _backup(self, final):
        """Keep the previous final under a temporary name; None if there is
        none. A hard link keeps final in place until it is replaced; where
        there are none (FAT), final is renamed aside."""
        if not os.path.lexists(final):
            return None
        dirname, base = os.path.split(final)
        old, fd = self._create(dirname or ".", base, "old")
        os.close(fd)
        os.unlink(old)
        try:
            os.link(final, old, follow_symlinks=False)
        except OSError:
            os.replace(final, old)
        return old

    def _commit(self):
        for tmp in self._staged.values():
            self._fsync(tmp)
        order = sorted(self._staged, key=lambda f: f == self.manifest_path)
        done = []
        try:
            for final in order:
                old = self._backup(final)
                done.append((final, old))
                os.replace(self._staged[final], final)
                del self._staged[final]
        except BaseException:
            for final, old in reversed(done):
                try:
                    if old is None:
                        if final not in self._staged:
                            os.unlink(final)
                    elif os.path.lexists(final) and os.path.samestat(
                            os.lstat(old), os.lstat(final)):
                        # final was not replaced and old is a link to it
                        # (rename() between two links does nothing).
                        os.unlink(old)
                    else:
                        os.replace(old, final)
                except OSError:
                    pass
            raise
        for d in sorted(set(os.path.dirname(f) or "." for f, _ in done)):
            self._fsync(d, directory=True)
        for _, old in done:
            if old is not None:
                try:
                    os.unlink(old)
                except OSError:
                    pass

    def abort(self):
        """Drop what was built; the files of an earlier package stay."""
        self.closed = True
        for f in (getattr(self, "tarfile", None), getattr(self, "cpiofile", None)):
            try:
                if f is not None:
                    f.close()
            except Exception:
                pass
        for tmp in getattr(self, "_staged", {}).values():
            try:
                os.unlink(tmp)
            except OSError:
                pass
        self._staged = {}
        self._release_lock()

    def close(self):
        if self.closed:
            return

        try:
            ti = tarfile.TarInfo("vendorfw/.vendorfw.manifest")
            ti.type = tarfile.REGTYPE
            fd = io.BytesIO()
            for i in self.manifest:
                fd.write(i.encode("ascii") + b"\n")
            ti.size = fd.tell()
            fd.seek(0)
            self.cpiofile.addfile(ti, fd)

            self.tarfile.close()
            self.cpiofile.close()

            self._write_staged(self.manifest_path,
                               "".join(i + "\n" for i in self.manifest).encode())
            for path, data in self._uboot:
                os.makedirs(os.path.dirname(path), exist_ok=True)
                self._write_staged(path, data)
            self._commit()
        except BaseException:
            self.abort()
            raise
        self.closed = True
        self._release_lock()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        if exc_type is None:
            self.close()
        else:
            self.abort()

    def add_file(self, name, data, mode=None):
        ti = tarfile.TarInfo(name)
        if name == "apple/audio-tuning/lsm2-param-fill.bin":
            ti.mode = 0o600
        if mode is not None:
            ti.mode = mode
        key = (data.sha, ti.mode)
        fd = None
        if key in self.hashes:
            ti.type = tarfile.LNKTYPE
            ti.linkname = self.hashes[key]
            self.manifest.append(f"LINK {name} {ti.linkname}")
        else:
            ti.type = tarfile.REGTYPE
            ti.size = len(data.data)
            fd = io.BytesIO(data.data)
            self.hashes[key] = name
            self.manifest.append(f"FILE {name} SHA256 {data.sha}")

        logging.info(f"+ {self.manifest[-1]}")
        self.tarfile.addfile(ti, fd)
        if fd is not None:
            fd.seek(0)
        ti.name = os.path.join("vendorfw", ti.name)
        if ti.linkname:
            ti.linkname = os.path.join("vendorfw", ti.linkname)
        self.cpiofile.addfile(ti, fd)

        if name in UBOOT_FILES:
            self._uboot.append((os.path.join(self.path, "u-boot", name), data.data))

    def add_files(self, it):
        for name, data in it:
            self.add_file(name, data)

    def __del__(self):
        # Only close() publishes: a package dropped part-way is discarded.
        if not getattr(self, "closed", True):
            self.abort()
