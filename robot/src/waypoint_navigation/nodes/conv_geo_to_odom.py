from pyproj import Proj, transform

def conv_geo_to_odom(in_geo, ref_geo, ref_odom):
    proj_utm = Proj(proj="utm", zone="32U", ellps="WGS84") # TODO: Change zone to Poland, Gdansk
    ref_cart = proj_utm(ref_geo[1], ref_geo[0]) # ref_cart[0] - Northing, ref_cart[1] - Easting
    in_cart = proj_utm(in_geo[1], in_geo[0])
    result = (in_cart[1] - ref_cart[1] + ref_odom[0], ref_cart[0] - in_cart[0] + ref_odom[1])
    return result
