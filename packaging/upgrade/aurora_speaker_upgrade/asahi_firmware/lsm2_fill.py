# SPDX-License-Identifier: MIT
"""Private, owner-local lsm2 parameter readback; no tuning defaults live here.

Only the public AudioUnit hosting interface is used. No rendering, audio
hardware, preferences, or system files are opened for writing. The exported
component factory is registered in this process, like any AUv2 plug-in, if
AudioComponentFindNext cannot find this system component.
"""
import hashlib
import json
import math
import plistlib
import struct
import sys
from xml.parsers.expat import ExpatError

NAME = "lsm2-param-fill.bin"
# 2: captures carry the component identity (unit_version may be 0, see
# read_parameters) and short presets may carry unit-published append-map IDs.
VERSION = 2
FORMAT = "json-id-name-f32bits-v1"
MAX_BYTES = 4 << 20
MAX_PARAMETERS = 16384
SUBTYPE = int.from_bytes(b"lsm2", "big")
COMPONENT = "/System/Library/Components/AudioDSP.component/Contents/MacOS/AudioDSP"


def base_ids():
    """The public compiler's 1,021-ID schema, never parameter values."""
    ids = set(range(3)) | set(range(1024, 1033)) | set(range(2048, 2062))
    for b in range(15):
        ids.update(range(3072 + 1024*b, 3109 + 1024*b))
    for t in range(3):
        ids.update(range(18432 + 1024*t, 18448 + 1024*t))
    for lo, hi in ((21504, 21512), (22528, 22537), (23552, 23561),
                   (1048576, 1048599), (1049600, 1049637),
                   (1050624, 1050645), (1051648, 1051669)):
        ids.update(range(lo, hi))
    for c in range(8):
        ids.update(range(1058816 + 1024*c, 1058849 + 1024*c))
    return ids


def extension_ids():
    """The compiler's exact append map (IDs only), as in speakerguardd's
    companion::extension_ids(). A saved ID outside base_ids() is accepted
    only when it is in this map and the running unit publishes it."""
    ids = {3, 21512}
    for b in range(15):
        ids.update(range(3109 + 1024*b, 3125 + 1024*b))
    for t in range(3):
        ids.update(range(18448 + 1024*t, 18450 + 1024*t))
    ids.update(range(24576, 24584))
    for b in (25600, 26624, 27648):
        ids.update(range(b, b + 4))
    ids.update(range(1048599, 1048604))
    for c in range(8):
        ids.add(1058849 + 1024*c)
    ids.update(range(1067008, 1067020))
    ids.update(range(1068032, 1068044))
    return ids


def missing_ids(saved):
    """Base IDs the preset does not carry. A preset is short exactly when
    this is non-empty, whatever its record count."""
    return base_ids() - saved


def extra_ids(saved):
    return saved - base_ids()


def _ids(ids, limit=8):
    ids = sorted(ids)
    return ", ".join(map(str, ids[:limit])) + (", ..." if len(ids) > limit else "")


def preset_ids(data):
    if not isinstance(data, bytes) or len(data) < 12:
        raise ValueError("lsm2 preset has no global parameter state")
    scope, element, count = struct.unpack_from(">III", data)
    if scope or element or count > MAX_PARAMETERS or len(data) != 12 + 8*count:
        raise ValueError("lsm2 requires one global parameter section")
    ids = [struct.unpack_from(">I", data, 12 + 8*i)[0] for i in range(count)]
    if len(set(ids)) != count:
        raise ValueError("duplicate lsm2 preset ID")
    return set(ids)


def presets(files):
    """(bundle-relative strip, effect, saved IDs) for every lsm2 strip."""
    for path, data in sorted(files.items()):
        if not path.endswith(".austrip"):
            continue
        try:
            strip = plistlib.loads(data)
        except (ValueError, plistlib.InvalidFileException, ExpatError):
            continue  # Unrelated/invalid strips are diagnosed by the compiler.
        for section in strip.get("strips", []):
            for effect in section.get("effects", []):
                if effect.get("unit", {}).get("subtype") == SUBTYPE:
                    yield path, effect, preset_ids(effect["aupreset"]["data"])


def needs_fill(files):
    return any(missing_ids(ids) for _, _, ids in presets(files))



def configuration_digest(files, aid):
    """Bind all source strips, propstrips and property plists, byte for byte.

    Selection metadata/calibration/readback are excluded; every AID/Common
    source file the collector can apply to an AU is included. This avoids
    reusing a stale capture after a propstrip or referenced plist changes.
    """
    h = hashlib.sha256(b"aurora-lsm2-configuration-v1\0")
    for path, data in sorted(files.items()):
        if path.startswith((aid + "/", "Common/")):
            h.update(path.encode() + b"\0")
            h.update(struct.pack("<Q", len(data)))
            h.update(data)
    return h.hexdigest()


def validate_configuration(data, files, aid):
    if json.loads(data).get("configuration_sha256") != configuration_digest(files, aid):
        raise ValueError("lsm2 fill source configuration changed")


def validate(data, aid, build, unit):
    if not isinstance(data, bytes) or not 0 < len(data) <= MAX_BYTES:
        raise ValueError("invalid lsm2 fill size")
    obj = json.loads(data)
    if (obj.get("version") != VERSION or obj.get("format") != FORMAT
            or obj.get("aid") != aid or obj.get("macos_build") != build
            or not unit or obj.get("unit_id") != unit):
        raise ValueError("lsm2 fill identity differs from its bundle")
    digest = obj.get("configuration_sha256")
    if not isinstance(digest, str) or len(digest) != 64 or any(c not in "0123456789abcdef" for c in digest):
        raise ValueError("invalid lsm2 fill configuration hash")
    captures = obj.get("presets")
    if not isinstance(captures, list) or not 0 < len(captures) <= 64:
        raise ValueError("invalid lsm2 fill capture list")
    seen = set()
    identities = set()
    for capture in captures:
        key = (capture["strip"], capture["preset_sha256"])
        version, component = capture["unit_version"], capture["component_sha256"]
        if (key in seen or type(version) is not int or not 0 <= version < 2**32
                or not _sha256(component)):
            raise ValueError("invalid lsm2 fill capture")
        seen.add(key)
        identities.add((version, component))
        records = capture["published"]
        if not isinstance(records, list) or not 0 < len(records) <= MAX_PARAMETERS:
            raise ValueError("invalid lsm2 published parameter list")
        ids = {}
        for r in records:
            # Names are provenance only and may be empty; only fill values
            # are used, so only they must be finite.
            if (type(r.get("id")) is not int or not 0 <= r["id"] < 2**32
                    or r["id"] in ids or not isinstance(r.get("name"), str)
                    or len(r["name"].encode()) > 1024
                    or type(r.get("bits")) is not int or not 0 <= r["bits"] < 2**32):
                raise ValueError("invalid lsm2 published parameter")
            ids[r["id"]] = r["bits"]
        fill = capture["fill_ids"]
        if (not isinstance(fill, list) or any(type(i) is not int for i in fill)
                or len(set(fill)) != len(fill) or not set(fill) <= ids.keys()):
            raise ValueError("invalid lsm2 fill IDs")
        if not all(math.isfinite(struct.unpack("<f", struct.pack("<I", ids[i]))[0]) for i in fill):
            raise ValueError("lsm2 fill value is not finite")
    if len(identities) != 1:
        raise ValueError("lsm2 fill captures name different units")
    return obj


def _sha256(value):
    return isinstance(value, str) and len(value) == 64 and all(c in "0123456789abcdef" for c in value)


def collect(files, aid, build, unit, readback=None, force=False):
    """Read all published parameters; only missing IDs may supply the fill.

    force permits a private full-preset dry run (empty fill_ids). Offline
    collection never invokes this function: a mounted macOS is not a live AU.

    A short preset may carry saved IDs beyond base_ids() (J713 carries one,
    21512). Each one must be in the compiler's append map and published by
    the running unit; any other ID is unknown and refused.
    """
    candidates = list(presets(files))
    if not any(force or missing_ids(ids) for _, _, ids in candidates):
        return None
    if not unit:
        raise ValueError("lsm2 fill requires the collecting Mac's unit identity")
    readback = readback or isolated_read_parameters
    captures = []
    for path, effect, saved in candidates:
        missing = missing_ids(saved)
        if not force and not missing:
            continue
        extra = extra_ids(saved)
        if missing:
            unknown = extra - extension_ids()
            if unknown:
                raise ValueError(f"short lsm2 preset carries {len(unknown)} saved ID(s) outside the "
                                 f"known lsm2 schema: {_ids(unknown)}")
        result = readback(path, effect, files)
        published = result["published"]
        ids = {r["id"] for r in published}
        if not missing <= ids:
            raise ValueError(f"AudioUnit did not publish {len(missing - ids)} missing lsm2 ID(s): "
                             f"{_ids(missing - ids)}")
        if missing and not extra <= ids:
            raise ValueError(f"AudioUnit does not publish {len(extra - ids)} saved lsm2 ID(s): "
                             f"{_ids(extra - ids)}")
        captures.append({"strip": path, "preset_sha256": hashlib.sha256(effect["aupreset"]["data"]).hexdigest(),
                         "unit_version": result["unit_version"], "component_sha256": result["component_sha256"],
                         "published": published, "fill_ids": sorted(missing)})
    obj = {"version": VERSION, "format": FORMAT, "aid": aid, "macos_build": build,
           "unit_id": unit, "configuration_sha256": configuration_digest(files, aid), "presets": captures}
    data = json.dumps(obj, sort_keys=True, separators=(",", ":")).encode()
    validate(data, aid, build, unit)
    return data


def isolated_read_parameters(path, effect, files):
    """Keep an AU crash or hang from taking the installer down with it.

    Private input/output travels over pipes; stderr is deliberately not
    forwarded, because a system component can print parameter values.
    """
    import subprocess
    request = plistlib.dumps({"path": path, "effect": effect, "files": files}, fmt=plistlib.FMT_BINARY)
    try:
        result = subprocess.run([sys.executable, "-I", "-B", __file__, "--readback"],
                                input=request, capture_output=True, timeout=60)
    except subprocess.TimeoutExpired:
        raise ValueError("lsm2 parameter readback timed out") from None
    if result.returncode:
        raise ValueError(f"lsm2 parameter readback failed (exit {result.returncode})")
    if len(result.stdout) > MAX_BYTES:
        raise ValueError("lsm2 parameter readback over budget")
    response = json.loads(result.stdout)
    return {k: response[k] for k in ("unit_version", "component_sha256", "published")}


def component_digest(path=COMPONENT):
    """SHA-256 of the system component executable that provides lsm2.

    Together with the macOS build this names the exact unit code that was
    read, also when the unit reports no version (see unit_version)."""
    h = hashlib.sha256()
    try:
        with open(path, "rb") as fd:
            for block in iter(lambda: fd.read(1 << 20), b""):
                h.update(block)
    except OSError as e:
        raise ValueError(f"lsm2 component unreadable: {e.strerror}") from None
    return h.hexdigest()


def unit_version(class_info, registry_version):
    """The version the running unit reports, or 0 when it reports none.

    Sources: ClassInfo "auversion" (the unit's own saved-state version, when
    the unit writes it) and the version of a component found in the system
    registry. ClassInfo "version" is the preset format version (0 on every
    unit and preset seen) and identifies nothing, so it is not used. When
    both sources report a version they must agree. 0 is a valid answer: the
    capture is then bound to the unit by the macOS build and the component
    SHA-256. A malformed or conflicting version is refused.
    """
    reported = class_info.get("auversion")
    if reported is not None and (type(reported) is not int or not 0 <= reported < 2**32):
        raise ValueError("lsm2 reported a malformed unit version")
    versions = {v for v in (reported or 0, registry_version) if v}
    if len(versions) > 1:
        raise ValueError("lsm2 ClassInfo and registry versions disagree")
    return versions.pop() if versions else 0


def read_parameters(path, effect, files):
    """Host the installed AU through public APIs, using ctypes (no compiler).

    Standard AUv2 factory ABI only; no private selectors or object layouts.
    Readback happens after ClassInfo and all matching propstrip properties.
    No AudioUnitRender/AudioOutputUnitStart call exists in this collector.
    """
    if sys.platform != "darwin":
        raise ValueError("lsm2 fill must be collected on the owner's running macOS")
    import ctypes as C
    u32, ptr = C.c_uint32, C.c_void_p
    au = C.CDLL("/System/Library/Frameworks/AudioToolbox.framework/AudioToolbox")
    cf = C.CDLL("/System/Library/Frameworks/CoreFoundation.framework/CoreFoundation")

    def api(lib, name, result, *args):
        f = getattr(lib, name)
        f.restype, f.argtypes = result, args
        return f

    find = api(au, "AudioComponentFindNext", ptr, ptr, ptr)
    getversion = api(au, "AudioComponentGetVersion", C.c_int32, ptr, C.POINTER(u32))
    register = api(au, "AudioComponentRegister", ptr, ptr, ptr, u32, ptr)
    new = api(au, "AudioComponentInstanceNew", C.c_int32, ptr, ptr)
    dispose = api(au, "AudioComponentInstanceDispose", C.c_int32, ptr)
    setprop = api(au, "AudioUnitSetProperty", C.c_int32, ptr, u32, u32, u32, ptr, u32)
    getprop = api(au, "AudioUnitGetProperty", C.c_int32, ptr, u32, u32, u32, ptr, ptr)
    propinfo = api(au, "AudioUnitGetPropertyInfo", C.c_int32, ptr, u32, u32, u32, ptr, ptr)
    getparam = api(au, "AudioUnitGetParameter", C.c_int32, ptr, u32, u32, u32, ptr)
    release = api(cf, "CFRelease", None, ptr)
    data_new = api(cf, "CFDataCreate", ptr, ptr, ptr, C.c_long)
    data_len = api(cf, "CFDataGetLength", C.c_long, ptr)
    data_ptr = api(cf, "CFDataGetBytePtr", ptr, ptr)
    parse = api(cf, "CFPropertyListCreateWithData", ptr, ptr, ptr, C.c_ulong, ptr, ptr)
    serialize = api(cf, "CFPropertyListCreateData", ptr, ptr, ptr, C.c_long, C.c_ulong, ptr)
    str_new = api(cf, "CFStringCreateWithCString", ptr, ptr, C.c_char_p, u32)
    str_get = api(cf, "CFStringGetCString", C.c_bool, ptr, ptr, C.c_long, u32)
    owned = []

    def check(rc):
        if rc:
            raise ValueError(f"lsm2 public AudioUnit API failed (OSStatus {rc})")

    def property_list(value):
        raw = plistlib.dumps(value, fmt=plistlib.FMT_BINARY)
        d = data_new(None, raw, len(raw))
        try:
            value = parse(None, d, 0, None, None)
            if not value:
                raise ValueError("could not create lsm2 property list")
            owned.append(value)
            return ptr(value)
        finally:
            release(d)

    def _class_info():
        ci, size = ptr(), u32(C.sizeof(ptr))
        check(getprop(instance, 0, 0, 0, C.byref(ci), C.byref(size)))
        if not ci:
            raise ValueError("lsm2 did not return ClassInfo")
        try:
            encoded = serialize(None, ci, 200, 0, None)
            if not encoded:
                raise ValueError("lsm2 ClassInfo cannot be serialized")
            try:
                return plistlib.loads(C.string_at(data_ptr(encoded), data_len(encoded)))
            finally:
                release(encoded)
        finally:
            release(ci)

    component_sha256 = component_digest()
    unit = effect["unit"]
    desc = (u32 * 5)(unit["type"], unit["subtype"], unit["manufacturer"], 0, 0)
    component = find(None, desc)
    registry_version = 0
    if component:
        found = u32()
        check(getversion(component, C.byref(found)))
        registry_version = found.value
    # This system effect may not be in the system's discoverable registry.
    # Register its standard exported AUv2 factory locally, never patch it.
    # Register's version argument is our placeholder, so a component
    # registered here never supplies a unit version.
    plugin = None
    if not component:
        plugin = C.CDLL("/System/Library/Components/AudioDSP.component/Contents/MacOS/AudioDSP")
        factory = C.cast(getattr(plugin, "AULoudspeakerManagerV2Factory"), ptr)
        label = str_new(None, b"Owner-local lsm2 collector", 0x08000100)
        try:
            component = register(desc, label, 0, factory)
        finally:
            release(label)
        if not component:
            raise ValueError("could not register the system lsm2 AudioUnit")
    instance = ptr()
    check(new(component, C.byref(instance)))
    try:
        preset = property_list(effect["aupreset"])
        check(setprop(instance, 0, 0, 0, C.byref(preset), C.sizeof(ptr)))
        prop_path = path[:-8] + ".propstrip"
        if prop_path in files:
            boxes = plistlib.loads(files[prop_path]).get("Boxes", [])
            for box in boxes:
                if box.get("Name") != effect.get("displayname"):
                    continue
                for p in box.get("Properties", []):
                    if "UInt32" in p:
                        v = u32(p["UInt32"])
                    elif "Path" in p:
                        ref = p["Path"]
                        prefix = "/System/Library/Audio/Tunings/"
                        if ref.startswith(prefix):
                            ref = ref[len(prefix):]
                        # The path is relative to the AID, as in the installer graph.
                        ref = ref if ref.startswith(("AID", "Common/")) else path.split("/")[0] + "/" + ref
                        if ref not in files:
                            raise ValueError("lsm2 property path is outside the private bundle")
                        v = property_list(plistlib.loads(files[ref]))
                    else:
                        raise ValueError("unsupported lsm2 propstrip property type")
                    number = p.get("Number", p.get("PropertyID"))
                    if not isinstance(number, (int, float)) or int(number) != number:
                        raise ValueError("invalid lsm2 property ID")
                    scope, element = p.get("Scope", 0), p.get("Element", 0)
                    check(setprop(instance, int(number), scope, element, C.byref(v), C.sizeof(v)))
        version = unit_version(_class_info(), registry_version)
        size = u32()
        check(propinfo(instance, 3, 0, 0, C.byref(size), None))
        if not size.value or size.value % 4 or size.value > MAX_PARAMETERS*4:
            raise ValueError("invalid lsm2 published parameter count")
        ids = (u32 * (size.value // 4))()
        check(getprop(instance, 3, 0, 0, ids, C.byref(size)))

        class ParameterInfo(C.Structure):
            _fields_ = [("name", C.c_char * 52), ("unitName", ptr), ("clumpID", u32), ("cfNameString", ptr),
                        ("unit", u32), ("min", C.c_float), ("max", C.c_float),
                        ("default", C.c_float), ("flags", u32)]

        published = []
        for ident in ids:
            info, size = ParameterInfo(), u32(C.sizeof(ParameterInfo))
            check(getprop(instance, 4, 0, ident, C.byref(info), C.byref(size)))
            try:
                name = info.name.split(b"\0", 1)[0].decode("utf-8", "replace")
                if info.cfNameString and info.flags & (1 << 27):
                    buf = C.create_string_buffer(1025)
                    if not str_get(info.cfNameString, buf, len(buf), 0x08000100):
                        raise ValueError("lsm2 parameter name is too long")
                    name = buf.value.decode("utf-8", "replace")
                value = C.c_float()
                check(getparam(instance, ident, 0, 0, C.byref(value)))
                published.append({"id": ident, "name": name,
                                  "bits": struct.unpack("<I", bytes(value))[0]})
            finally:
                # The SDK says to release cfNameString only with this flag.
                if info.flags & (1 << 4):
                    if info.cfNameString and info.flags & (1 << 27):
                        release(info.cfNameString)
                    if info.unitName and info.unit == 26:  # CustomUnit
                        release(info.unitName)
        return {"unit_version": version, "component_sha256": component_sha256, "published": published}
    finally:
        dispose(instance)
        for value in owned:
            release(value)


if __name__ == "__main__":
    # This entry point is an internal, pipe-only worker. No output paths,
    # logging or direct collection of another Mac's files are accepted.
    if sys.argv[1:] != ["--readback"]:
        raise SystemExit(2)
    try:
        request = plistlib.loads(sys.stdin.buffer.read(68 << 20))
        response = read_parameters(request["path"], request["effect"], request["files"])
        sys.stdout.buffer.write(json.dumps(response).encode())
    except Exception:
        raise SystemExit(1) from None
