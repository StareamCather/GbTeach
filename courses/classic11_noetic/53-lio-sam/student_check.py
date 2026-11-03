"""Check the scalar factor exercise separately from real GTSAM execution."""
import math

from learner import weighted_position, whitened_residual


def main():
    for x,z,s,r in ((1.,2.,.5,-2.),(3.,2.,.25,4.),(2.,2.,.1,0.)):
        if not math.isclose(whitened_residual(x,z,s),r,abs_tol=1e-12):
            raise ValueError("Check residual sign and divide by standard deviation")
    for observations,sigmas,expected in (([0.,10.],[1.,1.],5.),
                                         ([0.,10.],[1.,3.],1.),
                                         ([0.,10.],[3.,1.],9.),
                                         ([4.,4.,4.],[.5,2.,4.],4.)):
        if not math.isclose(weighted_position(observations,sigmas),expected,abs_tol=1e-12):
            raise ValueError("Use inverse variance, then divide by the sum of the weights")
    print("白化与加权估计练习通过。实际 iSAM2 因子图请使用桌面验收。")


if __name__=='__main__':
    main()
