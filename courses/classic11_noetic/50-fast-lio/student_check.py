"""An independent, offline check; it does not modify or score the estimator."""
import math

from learner import plane_residual, point_stamp_ns

def main():
    # A large epoch exposes rounding if either operand is converted to float.
    start = 1_703_000_000_000_000_007
    for offset in (0, 500_000, 99_500_000):
        stamp = point_stamp_ns(start, offset)
        if type(stamp) is not int or stamp != start+offset:
            raise ValueError("Acquisition stamp must preserve all integer nanoseconds")
    for p, q, n, expected in (
        ((9,2,0),(8,0,0),(1,0,0),1),
        ((8,10,0),(8,0,0),(1,0,0),0),
        ((7,2,0),(8,0,0),(1,0,0),-1),
        ((2,2,9),(0,0,0),(.6,.8,0),2.8),
    ):
        if not math.isclose(plane_residual(p,q,n),expected,abs_tol=1e-12):
            raise ValueError("Check the plane residual sign and normal projection")
    print("时间戳与点到面残差练习通过。估计器实机验收请使用桌面的验收按钮。")

if __name__=='__main__':
    main()
