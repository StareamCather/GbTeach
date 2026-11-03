"""Check numeric practice independently of the Point-LIO estimator."""
import math

from learner import event_rate_hz, group_sizes

def main():
    t=1_703_000_000_000_000_007
    fixtures=(([],[]),([t],[1]),([t,t,t+500_000,t+1_000_000,t+1_000_000,t+1_000_000],[2,1,3]))
    for stamps,expected in fixtures:
        if list(group_sizes(stamps))!=expected:
            raise ValueError("A new group starts only when the integer stamp changes")
    for stamps,rate in (([],0.),([t],0.),([t,t+500_000,t+1_500_000,t+2_000_000],1500.),
                        ([t,t+1],1e9)):
        if not math.isclose(event_rate_hz(stamps),rate,rel_tol=1e-10,abs_tol=1e-10):
            raise ValueError("Use the number of time intervals and subtract integer nanoseconds first")
    print("采样分组与输出率练习通过。实际估计器请使用桌面的验收按钮。")

if __name__=='__main__':
    main()
