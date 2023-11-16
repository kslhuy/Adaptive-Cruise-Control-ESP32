import numpy as np
steer_min = -1
steer_max = 1
steer_degree_min = 65
steer_degree_max = 115

def map_value_Steer_ESP2(x):





    steer_degree_calibarated_min = 20
    steer_degree_calibarated_max = 140

    pwm_calibrated_min = 138
    pwm_calibarated_max = 482
    
    mapped_value_1 = np.interp(x, (steer_min, steer_max), (steer_degree_min, steer_degree_max))
    # print(mapped_value_1)

    mapped_value_2 = np.interp(mapped_value_1, (steer_degree_calibarated_min, steer_degree_calibarated_max), (pwm_calibrated_min, pwm_calibarated_max)).round()

    # print(mapped_value_2)
    return mapped_value_2



def map_value_Steer_ESP1( x):
    pwm_calibrated_min = 92 
    pwm_calibrated_max = 486



    steer_degree_calibrated_min = 0
    steer_degree_calibrated_max = 130

    mapped_value_1 = np.interp(x, (steer_min, steer_max), (steer_degree_min, steer_degree_max))
    # print(mapped_value_1)
    mapped_value_2 = np.interp(mapped_value_1, (steer_degree_calibrated_min, steer_degree_calibrated_max), (pwm_calibrated_min, pwm_calibrated_max)).round()

    return mapped_value_2
# print(map_value_Steer_ESP2(0.9))

# print(round(0.7, 1))
# print(np.interp(120, (20, 140), (138, 482)).round())

# print(np.interp(100, (65, 115), (-1, 1)))
# print(np.interp(100, (65, 115), (-1, 1)))



# print(np.interp(90, (0, 130), (92, 486)).round())

# print(np.interp(100, (0, 130), (92, 486)).round())

print(map_value_Steer_ESP2(-1))
print(map_value_Steer_ESP2(1))
print(map_value_Steer_ESP2(0))





