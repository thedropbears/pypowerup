def target_objective(start_side, strategy, fms_switch, fms_scale):
    """A dictionary function that gives an output on the order of objectives based on inputs.
    The inputs are designated on the left in a tuple in the order of robot start side, strategy, switch side
    scale side. The outputs are given on the right in the order which the robot needs to go to"""
    objective_calculation = {
        # Double scale with right start
        ('R','double_scale','L','L'): ('cross_l_scale', 'same_l_scale'),
        ('R','double_scale','R','L'): ('cross_l_scale', 'same_l_scale'),
        ('R','double_scale','R','R'): ('same_r_scale', 'same_r_scale'),
        ('R','double_scale','L','R'): ('same_r_scale', 'same_r_scale'),
        # Double scale with left start
        ('L','double_scale','L','L'): ('same_l_scale', 'same_l_scale'),
        ('L','double_scale','R','L'): ('same_l_scale', 'same_l_scale'),
        ('L','double_scale','R','R'): ('cross_r_scale', 'same_r_scale'),
        ('L','double_scale','L','R'): ('cross_r_scale', 'same_r_scale'),
        # Switch and scale with right start
        ('R','switch_and_scale','L','R'): ('cross_l_scale', 'same_r_scale'),
        ('R','switch_and_scale','L','L'): ('cross_l_scale', 'same_l_switch'),
        ('R','switch_and_scale','R','L'): ('same_r_switch', 'cross_l_scale'),
        ('R','switch_and_scale','R','R'): ('same_r_switch', 'same_r_scale'),
        # Switch and scale with left start
        ('L','switch_and_scale','R','L'): ('same_l_scale', 'cross_r_switch'),
        ('L','switch_and_scale','R','R'): ('cross_r_scale', 'same_r_switch'),
        ('L','switch_and_scale','L','R'): ('same_l_switch', 'cross_r_scale'),
        ('L','switch_and_scale','L','L'): ('same_l_switch', 'same_l_scale')
    }
    print(objective_calculation[(start_side, strategy, fms_switch, fms_scale)])

target_objective('L', 'double_scale', 'R', 'L')