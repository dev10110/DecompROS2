
# represents a Polyhedron as P = { x : as[i] . x <= bs[i] for all i }
# represents a Polyhedron as P = { x : ns[i] . x <= ns[i].ps[i] for all i }
# so the normals point outwards

geometry_msgs/Point[] ps
geometry_msgs/Vector3[] ns
