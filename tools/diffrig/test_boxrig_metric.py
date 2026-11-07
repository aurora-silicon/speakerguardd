import pathlib,sys,numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).parent))
from boxrig import metric
x=np.ones((48000,2))*.1
assert metric(x,x,True)['abs_db']==0
assert abs(metric(x,x*10**(.051/20),False)['abs_db']-.051)<1e-10
assert abs(metric(x,x*10**(-.051/20),False)['abs_db']-.051)<1e-10
y=x.copy();y[480:960]*=10**(-.06/20)
assert metric(x,y,True)['abs_db']>.059999
assert metric(x,x*10**(.049/20),False)['abs_db']<.05
assert metric(np.zeros_like(x),x,False)['abs_db']>100
try:metric(x[:2],x,False)
except ValueError:pass
else:raise AssertionError('shape mismatch accepted')
y=x.copy();y[0,0]=np.nan
try:metric(x,y,False)
except ValueError:pass
else:raise AssertionError('nonfinite accepted')
print('metric controls PASS: ±0.051 fail, 0.049 passes, transient attenuation fails, silence, malformed/nonfinite')
