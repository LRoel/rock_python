#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

import math

G = {   'z': {'a': 1},
        'a': {'b': 1, 'f': 1, 'z': 1},
        'b': {'a': 1, 'c': 1},
        'c': {'b': 1, 'd': 1, 'h': 1},
        'd': {'c': 1, 'e': 1},
        'e': {'d': 1},
        'f': {'a': 1, 'g': 1, 'k': 1},
        'g': {'f': 1, 'h': 1},
        'h': {'c': 1, 'i': 1, 'g': 1, 'm': 1},
        'i': {'h': 1, 'j': 1},
        'j': {'i': 1},
        'k': {'p': 1, 'f': 1, 'l': 1},
        'l': {'m': 1, 'k': 1},
        'm': {'l': 1, 'n': 1, 'h': 1, 'r': 1},
        'n': {'o': 1, 'm': 1},
        'o': {'n': 1},
        'p': {'q': 1, 'k': 1},
        'q': {'r': 1, 'p': 1},
        'r': {'s': 1, 'm': 1, 'q': 1},
        's': {'t': 1, 'r': 1},
        't': {'s': 1}
     }

"""
0.61	0.61	0.67	0.67	0.63	2.22	2.23	2.27	2.24	2.24	3.78	3.75	3.7	3.73	3.76	5.39	5.36	5.38	5.38	5.38
2.2	    3.89	5.8	    7.35	8.76	2.31	4.02	5.71	7.31	8.73	2.34	4.12	5.7	7.31	8.76	2.31	4.19	5.74	7.31	8.7

"""
G_p = { 'a': (0.61, 2.20, 0),
        'b': (0.61, 3.89, 0),
        'c': (0.67, 5.80, 0),
        'd': (0.67, 7.35, 0),
        'e': (0.63, 8.76, 0),
        'f': (2.22, 2.31, 0),
        'g': (2.23, 4.02, 0),
        'h': (2.27, 5.71, 0),
        'i': (2.24, 7.31, 0),
        'j': (2.24, 8.73, 0),
        'k': (3.78, 2.34, 0),
        'l': (3.75, 4.12, 0),
        'm': (3.70, 5.70, 0),
        'n': (3.73, 7.31, 0),
        'o': (3.76, 8.76, 0),
        'p': (5.39, 2.31, 0),
        'q': (5.36, 4.19, 0),
        'r': (5.38, 5.74, 0),
        's': (5.38, 7.31, 0),
        't': (5.38, 8.70, 0),
        'z': (0.00, 1.50, 0)
       }

# print Dijkstra(G, 'a')
"""
public static double PointToSegDist(double x, double y, double x1, double y1, double x2, double y2)
{
double cross = (x2 - x1) * (x - x1) + (y2 - y1) * (y - y1);

if (cross <= 0) return Math.Sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1));

double d2 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
if (cross >= d2) return Math.Sqrt((x - x2) * (x - x2) + (y - y2) * (y - y2));

double r = cross / d2;
double px = x1 + (x2 - x1) * r;
double py = y1 + (y2 - y1) * r;
return Math.Sqrt((x - px) * (x - px) + (py - y) * (py - y));
}
"""


def point2segdist(x, y, x1, y1, x2, y2):
    cross = (x2 - x1) * (x - x1) + (y2 - y1) * (y - y1)
    if cross <= 0:
        dist = math.sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1))
        return dist
    d2 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1)
    if cross >= d2:
        dist =  math.sqrt((x - x2) * (x - x2) + (y - y2) * (y - y2))
        return dist
    r = cross / d2
    px = x1 + (x2 - x1) * r
    py = y1 + (y2 - y1) * r
    dist = math.sqrt((x - px) * (x - px) + (py - y) * (py - y))
    return dist


if __name__ == '__main__':
    try:
        x = 1.0
        y = 1.0
        c_dist = 999
        for v in G_p:
            for w in G[v]:
                dist = point2segdist(x, y, G_p[v][0], G_p[v][1], G_p[w][0], G_p[w][1])
                if dist <= c_dist:
                    c_dist = dist
                    c_seg = "线段" + v + w
                    # print G[v][w]
    except:
        print 'error'
