A = {'a': (0.61, 2.20, 0),
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

B = A
print B is A

B = A.copy()
print B is A