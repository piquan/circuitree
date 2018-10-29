#! /usr/bin/env python3

# Copyright 2018 Joel Ray Holveck
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


import math

def tessellate(mesh):
    for triangle in mesh:
        (a, b, c) = triangle
        ((ax, ay, az), (bx, by, bz), (cx, cy, cz)) = triangle
        abmid = ((ax+bx)/2, (ay+by)/2, (az+bz)/2)
        acmid = ((ax+cx)/2, (ay+cy)/2, (az+cz)/2)
        bcmid = ((bx+cx)/2, (by+cy)/2, (bz+cz)/2)
        yield (a, abmid, acmid)
        yield (abmid, b, bcmid)
        yield (acmid, bcmid, c)
        yield (abmid, bcmid, acmid)

def normalize(vector, r=1):
    (x, y, z) = vector
    norm = math.sqrt(x*x + y*y + z*z)
    return (x*r/norm, y*r/norm, z*r/norm)

def project_to_shell(mesh, r_inner=0.9, r_outer=1.0):
    for tri in mesh:
        (a, b, c) = tri
        a_inner = normalize(a, r_inner)
        a_outer = normalize(a, r_outer)
        b_inner = normalize(b, r_inner)
        b_outer = normalize(b, r_outer)
        c_inner = normalize(c, r_inner)
        c_outer = normalize(c, r_outer)
        points = [a_outer, b_outer, c_outer, a_inner, b_inner, c_inner]
        triangles = [[0, 1, 2],            # Outer face
                     [4, 1, 0], [4, 0, 3], # AB face
                     [3, 0, 2], [3, 2, 5], # AC face
                     [5, 2, 1], [5, 1, 4], # BC face
                     [3, 5, 4]]            # Inner face
        yield {"points": points, "triangles": triangles}

def scadize(obj):
    if type(obj) is tuple:
        return scadize(list(obj))
    if type(obj) is dict:
        parms = [k + "=" + scadize(v) for (k, v) in obj.items()]
        return ", ".join(parms)
    if type(obj) is list:
        vals = [scadize(x) for x in obj]
        return "[" + ", ".join(vals) + "]"
    if hasattr(obj, "__next__"):
        return scadize(list(obj))
    return repr(obj)

def main(r=13, wall=1):
    start_triangle = ((1, 0, 0), (0, 1, 0), (0, 0, 1))
    tris_planar = list(tessellate(tessellate([start_triangle])))
    outer_projections = project_to_shell(
        tris_planar, r_outer=r, r_inner=0)
    inner_projections = project_to_shell(
        tris_planar, r_outer=(r-wall), r_inner=0)
    print("module quarter_outer() {")
    for polyhedron in outer_projections:
        print("  polyhedron(" + scadize(polyhedron) + ");")
    print("}")
    print("module quarter_inner() {")
    for polyhedron in inner_projections:
        print("  polyhedron(" + scadize(polyhedron) + ");")
    print("}")
    
if __name__ == "__main__":
    main()
