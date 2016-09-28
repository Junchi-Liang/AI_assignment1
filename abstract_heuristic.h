#ifndef ABSTRACT_HEURISTIC_H
#define ABSTRACT_HEURISTIC_H

#include <iostream>
#include <cstdio>
#include <cstdlib>

#include "map_maker.h"

namespace abstract_heuristic_ns
{
	class abstract_heuristic
	{
		public:
			// return the h-value for the positon (cur_col, cur_row) while the goal position is (goal_col, goal_row) with the map, map_input
			virtual double h(int cur_col, int cur_row, int goal_col, int goal_row, const map_maker_ns::grid_map &map_input)
			{
				return 0;
			}
	};
}

#endif
