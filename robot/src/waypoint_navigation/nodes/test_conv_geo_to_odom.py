#!/usr/bin/env python
from waypoint_navigation_server import conv_geo_to_odom

def test_single_conv(in_geo, expected, geo_ref, odom_ref):
    result = conv_geo_to_odom(in_geo, geo_ref, odom_ref)
    print("odom = %f (%f[%f]), %f (%f[%f])" % (result[0], expected[0], (result[0]-expected[0])/expected[0]*100, result[1], expected[1], (result[1]-expected[1])/expected[1]*100))

def test_conv():
    geo_refs = (
        (49.899998370215485, 8.900000243996704),
        (49.900059815677295, 8.899926038771365),
        (49.89998484767207, 8.900066494436913)
    )
    odom_refs = (
        (0.125588970714454071, -0.00793456781548877),
        (7.233748723521987, 5.111974719152003),
        (-0.9266802440019012, -4.813575833743801)
    )
    for i in range(len(geo_refs)):
        test_single_conv((49.90008158778205, 8.900055637432734), (9.258129441861051, -3.9731864270136255), geo_refs[i], odom_refs[i])
        test_single_conv((49.899925385678934, 8.900043771216522), (-7.7698147312076316, -3.047976896220604), geo_refs[i], odom_refs[i])
        test_single_conv((49.899941994014085, 8.89993320265618), (-5.951942669191572, 4.779438298430898), geo_refs[i], odom_refs[i])
        test_single_conv((49.89994240335897, 8.90011080084284), (-5.786151777065306, -7.97497992169883), geo_refs[i], odom_refs[i])
        test_single_conv((49.900128080313735, 8.899832716182761), (14.811653527311073, 11.86251022448606), geo_refs[i], odom_refs[i])

if __name__ == "__main__":
    test_conv()   