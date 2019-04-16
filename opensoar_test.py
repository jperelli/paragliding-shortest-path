from aerofiles.igc import Reader
from opensoar.competition.soaringspot import get_info_from_comment_lines
from opensoar.task.trip import Trip
from opensoar.task.waypoint import Waypoint
from opensoar.task.race_task import RaceTask
import json

import pyproj
import json
from shapely.geometry import Point, LineString, GeometryCollection
from functools import partial
from shapely.ops import transform
from shapely_geojson import dumps, Feature, FeatureCollection
from geographiclib.geodesic import Geodesic

with open('race_task_completed.igc', 'r') as f:
    parsed_igc_file = Reader().read(f)

# example.igc comes from soaringspot and contains task inforamtion
#task, _, _ = get_info_from_comment_lines(parsed_igc_file)
#_, trace = parsed_igc_file['fix_records']
#
#trip = Trip(task, trace)
#task_distance_covered = sum(trip.distances)
#
#print (task_distance_covered)

with open('task_2018-08-16.json', 'r') as f:
    jobj = json.loads(f.read())

waypoint_list = []
print ("\nWaypoint list:")
for turnpoint in jobj['turnpoints']:
    lon = turnpoint['waypoint']['lon']
    lat = turnpoint['waypoint']['lat']
    name = turnpoint['waypoint']['name']
    r_min = 0
    r_max = turnpoint['radius']
    # Cylindric area
    a_min = 0.0
    a_max = 180.0
    s_line = False
    sector_orientation = 'fixed'
    distance_correction = None
    orientation_angle = 0

    # if 'type' in turnpoint:
    #     if turnpoint['type'].lower() == 'sss':
    #         if jobj['sss']['direction'] == 'ENTER':
    #             print ("ENTER CYLINDER DETECTED")
    #             r_min = r_max
    #             r_max = 10000000

    w = Waypoint(name,
                 lat, lon,
                 r_min, a_min,
                 r_max, a_max,
                 s_line,
                 sector_orientation,
                 distance_correction,
                 orientation_angle)
    waypoint_list.append(w)

#waypoint_list = waypoint_list[0:6]


r = RaceTask(waypoint_list)

d = (r.calculate_task_distances())

for i in range(len(waypoint_list)):
    w = waypoint_list[i]
    print (w.latitude, w.longitude, w.r_max, 'm')
    if i != len(waypoint_list) - 1:
        cum_dist = sum(d[:i])
        #print ("{:0.3f}km".format(cum_dist/1000))

print ("\nDistances list:")
print (d)

print ("\nTotal distance:")
print (sum(d))

def circle(point, rad):
    """
        Create a circle of radius rad in meters around a shapely point
        code from: https://gis.stackexchange.com/q/268250/6900
    """
    local_azimuthal_projection = f"+proj=aeqd +R=6371000 +units=m +lat_0={point.y} +lon_0={point.x}"
    wgs84_to_aeqd = partial(
        pyproj.transform,
        pyproj.Proj('+proj=longlat +datum=WGS84 +no_defs'),
        pyproj.Proj(local_azimuthal_projection),
    )
    aeqd_to_wgs84 = partial(
        pyproj.transform,
        pyproj.Proj(local_azimuthal_projection),
        pyproj.Proj('+proj=longlat +datum=WGS84 +no_defs'),
    )
    point_transformed = transform(wgs84_to_aeqd, point)
    buffer = point_transformed.buffer(rad)
    buffer_wgs84 = transform(aeqd_to_wgs84, buffer)
    return buffer_wgs84

pairs = []
for pair in zip(waypoint_list[::1], waypoint_list[1::1]):
    w0 = pair[0]
    w1 = pair[1]
    p0 = Point(w0.longitude, w0.latitude)
    p1 = Point(w1.longitude, w1.latitude)
    circle_a = circle(p0, w0.r_max)
    circle_b = circle(p1, w1.r_max)
    line_centers = LineString([p0, p1])
    line = line_centers.difference(circle_a).difference(circle_b)
    pairs.append({
        'line': line,
        'A': circle_a,
        'B': circle_b,
    })

ITERATIONS = 10

for I in range(ITERATIONS):
    for j in range(1, len(waypoint_list) - 2):
        i = j + 1
        p1 = pairs[i - 1]
        p2 = pairs[i]
        circle_i = p2['A']
        segment = LineString([p1['line'].coords[1], p2['line'].coords[0]])
        midpoint = segment.interpolate(0.5, normalized=True)
        p1['line'] = LineString([p1['line'].coords[0], midpoint]).difference(circle_i)
        p2['line'] = LineString([midpoint, p2['line'].coords[1]]).difference(circle_i)


points = []
for p in pairs:
    points.append(Point(p['line'].coords[0]))
    points.append(Point(p['line'].coords[1]))

path = LineString(points) # this is the final result

dist = 0
geod = Geodesic.WGS84
for pair in zip(points[::1], points[1::1]):
    dist = dist + geod.Inverse(pair[0].y, pair[0].x, pair[1].y, pair[1].x)['s12']
print("DISTANCE (m): ", dist)

# printing results
circles = [circle(Point(w.longitude, w.latitude), w.r_max) for w in waypoint_list]
print(dumps(FeatureCollection([Feature(f) for f in (circles + [path])]), indent=2))
