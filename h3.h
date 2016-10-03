#ifndef H3_H
#define H3_H

#include "abstract_heuristic.h"

namespace heuristic_ns
{
	// assume the distribution of highways in neighbors is the same as the distribution in the whole map
	// assume the distribution of hard-to-traverse in neighbors is the same as the distribution in the whole map
	// scan the neighbors and get coverage of highways, beta, and coverage of hard-to-traverse, alpha
	// h = euclidean distance * (alpha + 2 * (1 - alpha)) * (0.25 * beta + 1 * (1 - beta))
	class h3: public abstract_heuristic
	{
		public:
		// return the h-value for the positon (cur_col, cur_row) while the goal position is (goal_col, goal_row) with the map, map_input
		double h(int cur_col, int cur_row, int goal_col, int goal_row, const map_maker_ns::grid_map &map_input)
		{
			int i, d_col = cur_col - goal_col, d_row = cur_row - goal_row;
			double alpha = 0, beta = 0, res;
			for (i = 0; i < map_maker_ns::MAX_DIR; i++)
			{
				int next_col = cur_col + map_maker_ns::DELTA_COL[i], next_row = cur_row + map_maker_ns::DELTA_ROW[i];
				if (map_input.check_available(next_col, next_row))
				{
					char bit_read = map_input.read_bit(next_col, next_row);
					if (bit_read == map_maker_ns::UNBLOCKED || bit_read == map_maker_ns::UNBLOCKED_HIGHWAY)
						alpha = alpha + 1;
					if (bit_read == map_maker_ns::UNBLOCKED_HIGHWAY || bit_read == map_maker_ns::HARD_HIGHWAY)
						beta = beta + 1;
				}
			}
			alpha = alpha / 8.0;
			beta = beta / 8.0;
			res = sqrt(d_col * d_col + d_row * d_row);
			return res * (alpha + 2 * (1 - alpha)) * (0.25 * beta + 1 * (1 - beta));
		}
	};
}

#endif
