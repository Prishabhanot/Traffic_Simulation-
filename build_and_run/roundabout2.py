from trafficSimulator import *
import numpy as np


class Intersection:
    def __init__(self):
        self.sim = Simulation()
        lane_space = 3.5
        intersection_size = 49
        island_width = 2
        length = 43.75 #I have shortened the length of the entrance roads because vehicles base speeds are much lower because they are driving in a round about, however they would be able to drive faster in the entrance.
        radius = 18

        self.v = 8.5

        # entrance 0-7
        self.sim.create_segment((lane_space/2 + island_width/2, length + intersection_size/2), (lane_space/2 + island_width/2, intersection_size/2)) 
        self.sim.create_segment((length + intersection_size/2, -lane_space/2 - island_width/2), (intersection_size/2, -lane_space/2 - island_width/2)) 
        self.sim.create_segment((-lane_space/2 - island_width/2, -length - intersection_size/2), (-lane_space/2 - island_width/2, - intersection_size/2)) 
        self.sim.create_segment((-length - intersection_size/2, lane_space/2 + island_width/2), (-intersection_size/2, lane_space/2 + island_width/2)) 
        self.sim.create_segment((lane_space + island_width/2, length + intersection_size/2), (lane_space + island_width/2, intersection_size/2)) 
        self.sim.create_segment((length + intersection_size/2, -lane_space - island_width/2), (intersection_size/2, -lane_space - island_width/2)) 
        self.sim.create_segment((-lane_space - island_width/2, -length - intersection_size/2), (-lane_space - island_width/2, - intersection_size/2)) 
        self.sim.create_segment((-length - intersection_size/2, lane_space + island_width/2), (-intersection_size/2, lane_space + island_width/2)) 
        # exit 8-15
        self.sim.create_segment((-lane_space/2 - island_width/2, intersection_size/2), (-lane_space/2 - island_width/2, length + intersection_size/2))
        self.sim.create_segment((intersection_size/2, lane_space/2 + island_width/2), (length+intersection_size/2, lane_space/2 + island_width/2))
        self.sim.create_segment((lane_space/2 + island_width/2, -intersection_size/2), (lane_space/2 + island_width/2, -length - intersection_size/2))
        self.sim.create_segment((-intersection_size/2, -lane_space/2 - island_width/2), (-length-intersection_size/2, -lane_space/2 - island_width/2))
        self.sim.create_segment((-lane_space - island_width/2, intersection_size/2), (-lane_space - island_width/2, length + intersection_size/2))
        self.sim.create_segment((intersection_size/2, lane_space + island_width/2), (length+intersection_size/2, lane_space + island_width/2))
        self.sim.create_segment((lane_space + island_width/2, -intersection_size/2), (lane_space + island_width/2, -length - intersection_size/2))
        self.sim.create_segment((-intersection_size/2, -lane_space - island_width/2), (-length-intersection_size/2, -lane_space - island_width/2))
        # corners 16-23
        self.sim.create_quadratic_bezier_curve((lane_space + island_width/2, radius),(radius,radius),(radius,lane_space + island_width/2))
        self.sim.create_quadratic_bezier_curve((radius,-lane_space - island_width/2),(radius,-radius),(lane_space + island_width/2,-radius))
        self.sim.create_quadratic_bezier_curve((-lane_space - island_width/2,-radius),(-radius,-radius),(-radius,-lane_space - island_width/2))
        self.sim.create_quadratic_bezier_curve((-radius,lane_space + island_width/2),(-radius,radius),(-lane_space - island_width/2, radius))
        self.sim.create_quadratic_bezier_curve((lane_space*2 + island_width/2, radius),(radius*2,radius*2),(radius*2,lane_space*2 + island_width/2))
        self.sim.create_quadratic_bezier_curve((radius*2,-lane_space*2 - island_width/2),(radius*2,-radius*2),(lane_space*2 + island_width/2,-radius*2))
        self.sim.create_quadratic_bezier_curve((-lane_space*2 - island_width/2,-radius*2),(-radius*2,-radius*2),(-radius*2,-lane_space*2 - island_width/2))
        self.sim.create_quadratic_bezier_curve((-radius*2,lane_space*2 + island_width/2),(-radius*2,radius*2),(-lane_space*2 - island_width/2, radius*2))
        # connectors 24-31
        self.sim.create_segment((radius,lane_space + island_width/2),(radius,-lane_space - island_width/2))
        self.sim.create_segment((lane_space + island_width/2,-radius),(-lane_space - island_width/2,-radius))
        self.sim.create_segment((-radius,-lane_space - island_width/2),(-radius,lane_space + island_width/2))
        self.sim.create_segment((-lane_space - island_width/2, radius),(lane_space + island_width/2, radius))
        self.sim.create_segment((radius*2,lane_space*2 + island_width/2),(radius*2,-lane_space*2 - island_width/2))
        self.sim.create_segment((lane_space*2 + island_width/2,-radius*2),(-lane_space*2 - island_width/2,-radius*2))
        self.sim.create_segment((-radius*2,-lane_space*2 - island_width/2),(-radius*2,lane_space*2 + island_width/2))
        self.sim.create_segment((-lane_space*2 - island_width/2, radius*2),(lane_space*2 + island_width/2, radius*2))
        # turn into corners 32-39
        self.sim.create_quadratic_bezier_curve((lane_space/2 + island_width/2, intersection_size/2), (lane_space/2 + island_width/2, radius), (lane_space + island_width/2, radius))
        self.sim.create_quadratic_bezier_curve((intersection_size/2, -lane_space/2 - island_width/2), (radius, -lane_space/2 - island_width/2), (radius, -lane_space - island_width/2))
        self.sim.create_quadratic_bezier_curve((-lane_space/2 - island_width/2, - intersection_size/2), (-lane_space/2 - island_width/2, -radius), (-lane_space - island_width/2, -radius))
        self.sim.create_quadratic_bezier_curve((-intersection_size/2, lane_space/2 + island_width/2), (-radius, lane_space/2 + island_width/2), (-radius, lane_space + island_width/2))
        self.sim.create_quadratic_bezier_curve((lane_space + island_width/2, intersection_size/2), (lane_space + island_width/2, radius*2), (lane_space*2 + island_width/2, radius*2))
        self.sim.create_quadratic_bezier_curve((intersection_size/2, -lane_space - island_width/2), (radius*2, -lane_space - island_width/2), (radius*2, -lane_space*2 - island_width/2))
        self.sim.create_quadratic_bezier_curve((-lane_space - island_width/2, - intersection_size/2), (-lane_space - island_width/2, -radius*2), (-lane_space*2 - island_width/2, -radius*2))
        self.sim.create_quadratic_bezier_curve((-intersection_size/2, lane_space + island_width/2), (-radius*2, lane_space + island_width/2), (-radius*2, lane_space*2 + island_width/2))
        # turn to exit 40-47
        self.sim.create_quadratic_bezier_curve((radius, lane_space + island_width/2), (radius, lane_space/2 + island_width/2), (intersection_size/2, lane_space/2 + island_width/2))
        self.sim.create_quadratic_bezier_curve((lane_space + island_width/2, -radius), (lane_space/2 + island_width/2, -radius), (lane_space/2 + island_width/2, -intersection_size/2))
        self.sim.create_quadratic_bezier_curve((-radius, -lane_space - island_width/2), (-radius, -lane_space/2 - island_width/2), (-intersection_size/2, -lane_space/2 - island_width/2))
        self.sim.create_quadratic_bezier_curve((-lane_space - island_width/2, radius), (-lane_space/2 - island_width/2, radius), (-lane_space/2 - island_width/2, intersection_size/2))
        self.sim.create_quadratic_bezier_curve((radius*2, lane_space*2 + island_width/2), (radius*2, lane_space + island_width/2), (intersection_size/2, lane_space + island_width/2))
        self.sim.create_quadratic_bezier_curve((lane_space*2 + island_width/2, -radius*2), (lane_space + island_width/2, -radius*2), (lane_space + island_width/2, -intersection_size/2))
        self.sim.create_quadratic_bezier_curve((-radius*2, -lane_space*2 - island_width/2), (-radius*2, -lane_space - island_width/2), (-intersection_size/2, -lane_space - island_width/2))
        self.sim.create_quadratic_bezier_curve((-lane_space*2 - island_width/2, radius*2), (-lane_space - island_width/2, radius*2), (-lane_space - island_width/2, intersection_size/2))

        # Define interfering paths
        self.sim.define_interfearing_paths([0, 32], [8, 40], turn=True)
        self.sim.define_interfearing_paths([1, 33], [9, 41], turn=True)
        self.sim.define_interfearing_paths([2, 34], [10, 42], turn=True)
        self.sim.define_interfearing_paths([3, 35], [11, 43], turn=True)
        self.sim.define_interfearing_paths([4, 36], [12, 44], turn=True)
        self.sim.define_interfearing_paths([5, 37], [13, 45], turn=True)
        self.sim.define_interfearing_paths([6, 38], [14, 46], turn=True)
        self.sim.define_interfearing_paths([7, 39], [15, 47], turn=True)
        self.sim.define_interfearing_paths([16, 24], [17, 25], turn=True)
        self.sim.define_interfearing_paths([18, 26], [19, 27], turn=True)
        self.sim.define_interfearing_paths([20, 28], [21, 29], turn=True)
        self.sim.define_interfearing_paths([22, 30], [23, 31], turn=True)

        self.vg = VehicleGenerator({
            'vehicles': [
                (1, {'path': [0, 32, 16, 40, 9], 'v_max': self.v}),
                (1, {'path': [0, 32, 16, 24, 17, 41, 10], 'v_max': self.v}),
                (1, {'path': [0, 32, 16, 24, 17, 25, 18, 42, 11], 'v_max': self.v}),
                (1, {'path': [0, 32, 16, 24, 17, 25, 18, 26, 19, 43, 8], 'v_max': self.v}),
                (1, {'path': [1, 33, 17, 41, 10], 'v_max': self.v}),
                (1, {'path': [1, 33, 17, 25, 18, 42, 11], 'v_max': self.v}),
                (1, {'path': [1, 33, 17, 25, 18, 26, 19, 43, 8], 'v_max': self.v}),
                (1, {'path': [1, 33, 17, 25, 18, 26, 19, 27, 16, 40, 9], 'v_max': self.v}),
                (1, {'path': [2, 34, 18, 42, 11], 'v_max': self.v}),
                (1, {'path': [2, 34, 18, 26, 19, 43, 8], 'v_max': self.v}),
                (1, {'path': [2, 34, 18, 26, 19, 27, 16, 40, 9], 'v_max': self.v}),
                (1, {'path': [2, 34, 18, 26, 19, 27, 16, 24, 17, 41, 10], 'v_max': self.v}),
                (1, {'path': [3, 35, 19, 43, 8], 'v_max': self.v}),
                (1, {'path': [3, 35, 19, 27, 16, 40, 9], 'v_max': self.v}),
                (1, {'path': [3, 35, 19, 27, 16, 24, 17, 41, 10], 'v_max': self.v}),
                (1, {'path': [3, 35, 19, 27, 16, 24, 17, 25, 18, 42, 11], 'v_max': self.v}),
                (1, {'path': [4, 36, 20, 44, 13], 'v_max': self.v}),
                (1, {'path': [4, 36, 20, 28, 21, 45, 14], 'v_max': self.v}),
                (1, {'path': [4, 36, 20, 28, 21, 29, 22, 46, 15], 'v_max': self.v}),
                (1, {'path': [4, 36, 20, 28, 21, 29, 22, 30, 23, 47, 12], 'v_max': self.v}),
                (1, {'path': [5, 37, 21, 45, 14], 'v_max': self.v}),
                (1, {'path': [5, 37, 21, 29, 22, 46, 15], 'v_max': self.v}),
                (1, {'path': [5, 37, 21, 29, 22, 30, 23, 47, 12], 'v_max': self.v}),
                (1, {'path': [5, 37, 21, 29, 22, 30, 23, 31, 20, 44, 13], 'v_max': self.v}),
                (1, {'path': [6, 38, 22, 46, 15], 'v_max': self.v}),
                (1, {'path': [6, 38, 22, 30, 23, 47, 12], 'v_max': self.v}),
                (1, {'path': [6, 38, 22, 30, 23, 31, 20, 44, 13], 'v_max': self.v}),
                (1, {'path': [6, 38, 22, 30, 23, 31, 20, 28, 21, 45, 14], 'v_max': self.v}),
                (1, {'path': [7, 39, 23, 47, 12], 'v_max': self.v}),
                (1, {'path': [7, 39, 23, 31, 20, 44, 13], 'v_max': self.v}),
                (1, {'path': [7, 39, 23, 31, 20, 28, 21, 45, 14], 'v_max': self.v}),
                (1, {'path': [7, 39, 23, 31, 20, 28, 21, 29, 22, 46, 15], 'v_max': self.v}),
            ], 'vehicle_rate': 30
        }
        
        )

                # Define interfering paths
        self.sim.define_interfearing_paths([0, 16], [4, 20], turn=True)
        self.sim.define_interfearing_paths([1, 17], [5, 21], turn=True)
        self.sim.define_interfearing_paths([2, 18], [6, 22], turn=True)
        self.sim.define_interfearing_paths([3, 19], [7, 23], turn=True)
        self.sim.define_interfearing_paths([8, 12], [9, 13], turn=True)
        self.sim.define_interfearing_paths([10, 14], [11, 15], turn=True)
        # self.sim.define_interfearing_paths([0,16],[15,8],turn=True)
        # self.sim.define_interfearing_paths([1,17],[12,9],turn=True)
        # self.sim.define_interfearing_paths([2,18],[13,10],turn=True)
        # self.sim.define_interfearing_paths([3,19],[14,11],turn=True)
        self.sim.add_vehicle_generator(self.vg)
    
    def get_sim(self):
        return self.sim