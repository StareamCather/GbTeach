def controller(target, position, velocity):
    # 填写误差；速度为正时阻尼应为负
    error = None
    damping = None
    if error is None or damping is None:
        raise NotImplementedError("填写误差和阻尼")
    return max(-10,min(10,18*error+damping))
