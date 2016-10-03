#ifndef H2_H
#define H2_H

#include "abstract_heuristic.h"

namespace heuristic_ns
{
	// assume highways are everywhere, and assume we can use highways diagonally
	// admissiable and consistant
	class h2: public abstract_heuristic
	{
		public:
			// return the h-value for the positon (cur_col, cur_row) while the goal position is (goal_col, goal_row) with the map, map_input
			double h(int cur_col, int cur_row, int goal_col, int goal_row, const map_maker_ns::grid_map &map_input)
			{
				double d_col = cur_col - goal_col;
				double d_row = cur_row - goal_row;
				return 0.25 * sqrt(d_col * d_col + d_row * d_row);
			}
	};
}

#endif
