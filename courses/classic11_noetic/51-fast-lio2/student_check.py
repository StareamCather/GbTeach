"""Check the personal numeric exercise independently from the real estimator."""
import math

from learner import translation_jacobian, voxel_key


def main():
    for p, size, expected in (
        ((.49,.50,-.01),.5,(0,1,-1)),
        ((-.5,-.501,1.01),.5,(-1,-2,2)),
        ((0,0,0),.5,(0,0,0)),
    ):
        result=tuple(voxel_key(p,size))
        if result!=expected or any(type(x) is not int for x in result):
            raise ValueError("Use floor for negative cell coordinates; return integers")
    p=(2.,3.,4.)
    q=(1.,0.,2.)
    n=(.6,0.,-.8)
    step=1e-5
    expected=[]
    for axis in range(3):
        def residual(delta):
            return sum(ni*(x-y+(delta if i==axis else 0.)) for i,(ni,x,y) in enumerate(zip(n,p,q)))
        expected.append((residual(step)-residual(-step))/(2*step))
    result=tuple(translation_jacobian(n))
    if len(result)!=3 or any(not math.isclose(a,b,abs_tol=1e-9) for a,b in zip(result,expected)):
        raise ValueError("The translation derivative must match the world-frame finite difference")
    print("体素索引与平移雅可比练习通过。实际估计器请使用桌面的验收按钮。")


if __name__=='__main__':
    main()
