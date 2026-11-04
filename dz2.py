import numpy as np
f32=np.float32
target={48000:1.0060077,44100:0.9998977,96000:1.0420919}
def design(f0,Q,fs,prec):
    if prec=="f32":
        w=f32(f32(2*np.pi)*f32(f0)/f32(fs)); al=f32(np.sin(w,dtype=np.float32)/f32(2*Q)); c=np.cos(w,dtype=np.float32); a0=f32(1)+al
        b0=f32(f32(f32(1)-c)/f32(2))/a0; a1=f32(f32(-2)*c)/a0; a2=f32(f32(1)-al)/a0
    else:
        w=2*np.pi*f0/fs; al=np.sin(w)/(2*Q); c=np.cos(w); a0=1+al
        b0,a1,a2=f32((1-c)/2/a0),f32(-2*c/a0),f32((1-al)/a0)
    return f32(b0),f32(a1),f32(a2)
def run(b0,a1,a2,form,n):
    x=f32(1); 
    if form=="df1":
        x1=x2=y1=y2=f32(0)
        for k in range(n):
            y=f32(f32(f32(f32(b0*x)+f32(f32(2)*f32(b0*x1)))+f32(b0*x2))-f32(a1*y1))-f32(a2*y2)
            x2,x1=x1,x; y2,y1=y1,y
    elif form=="df1b":  # b0*(x+2x1+x2)
        x1=x2=y1=y2=f32(0)
        for k in range(n):
            y=f32(f32(b0*f32(f32(x+f32(2)*x1)+x2))-f32(a1*y1))-f32(a2*y2)
            x2,x1=x1,x; y2,y1=y1,y
    elif form=="df2":
        w1=w2=f32(0)
        for k in range(n):
            w=f32(f32(x-f32(a1*w1))-f32(a2*w2)); y=f32(b0*f32(f32(w+f32(2)*w1)+w2)); w2,w1=w1,w
    elif form=="tdf2":
        s1=s2=f32(0); b1=f32(2*b0)
        for k in range(n):
            y=f32(b0*x+s1); s1=f32(f32(b1*x-f32(a1*y))+s2); s2=f32(f32(b0*x)-f32(a2*y))
    return float(y)
for prec in ("f64","f32"):
  for form in ("df1","df1b","df2","tdf2"):
    for f0,Q in ((20,0.65),(19.95,0.65),(20,0.649)):
        res={fs: run(*design(f0,Q,fs,prec),form,fs*4) for fs in target}
        err=max(abs(res[fs]-target[fs]) for fs in target)
        print(prec,form,f0,Q,{k:round(v,7) for k,v in res.items()},"maxerr %.2e"%err)
