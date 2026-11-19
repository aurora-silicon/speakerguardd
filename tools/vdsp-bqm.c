// SW-TMST5 black-box probe of the public Accelerate vDSP_biquadm (float) API. Not tainted: our own program.
// usage: bqm COEFS.txt IN.f32 OUT.f32 CHANNELS CALLLEN...   (COEFS.txt: M lines "b0 b1 b2 a1 a2", same for every channel)
// IN/OUT: interleaved float32, CHANNELS channels. Call lengths repeat cyclically until the input ends.
#include <Accelerate/Accelerate.h>
#include <stdio.h>
#include <stdlib.h>
int main(int argc, char **argv) {
    if (argc < 6) { fprintf(stderr, "usage\n"); return 2; }
    FILE *cf = fopen(argv[1], "r"); double c[5 * 64]; int m = 0;
    while (m < 64 && fscanf(cf, "%lf %lf %lf %lf %lf", &c[5*m], &c[5*m+1], &c[5*m+2], &c[5*m+3], &c[5*m+4]) == 5) m++;
    fclose(cf);
    int ch = atoi(argv[4]); int ncalls = argc - 5; int *calls = malloc(sizeof(int) * ncalls);
    for (int i = 0; i < ncalls; i++) calls[i] = atoi(argv[5 + i]);
    double *co = malloc(sizeof(double) * 5 * m * ch);
    for (int s = 0; s < m; s++) for (int k = 0; k < ch; k++) for (int j = 0; j < 5; j++) co[(s * ch + k) * 5 + j] = getenv("BQM_DOUBLE") ? c[5 * s + j] : (double)(float)c[5 * s + j];
    vDSP_biquadm_Setup st = vDSP_biquadm_CreateSetup(co, m, ch);
    FILE *fi = fopen(argv[2], "rb"); fseek(fi, 0, SEEK_END); long n = ftell(fi) / 4 / ch; fseek(fi, 0, SEEK_SET);
    float *in = malloc(4 * n * ch), *out = malloc(4 * n * ch); fread(in, 4, n * ch, fi); fclose(fi);
    float **xs = malloc(sizeof(float*) * ch), **ys = malloc(sizeof(float*) * ch);
    float *xb = malloc(4 * 8192 * ch), *yb = malloc(4 * 8192 * ch);
    long pos = 0; int ci = 0;
    while (pos < n) {
        long len = calls[ci % ncalls]; ci++; if (pos + len > n) len = n - pos;
        for (int k = 0; k < ch; k++) { xs[k] = xb + k * 8192; ys[k] = yb + k * 8192; for (long i = 0; i < len; i++) xs[k][i] = in[(pos + i) * ch + k]; }
        vDSP_biquadm(st, (const float **)xs, 1, ys, 1, len);
        for (int k = 0; k < ch; k++) for (long i = 0; i < len; i++) out[(pos + i) * ch + k] = ys[k][i];
        pos += len;
    }
    FILE *fo = fopen(argv[3], "wb"); fwrite(out, 4, n * ch, fo); fclose(fo);
    vDSP_biquadm_DestroySetup(st);
    return 0;
}
