#ifndef H5_H
#define H5_H

#include "abstract_heuristic.h"

namespace heuristic_ns
{
	// assume the distribution of highways in neighbors is the same as the distribution in the whole map
	// assume the distribution of unblocked cells in neighbors is the same as the distribution in the whole map
	// sample the neighbors and get coverage of highways, beta, and coverage of unblocked cells, alpha
	// h = (1 - beta) * euclidean distance * (alpha + 2 * (1 - alpha)) + 0.25 * beta * manhattan distance * (alpha + 2 * (1 - alpha))
	class h5: public abstract_heuristic
	{
		public:
		// return the h-value for the positon (cur_col, cur_row) while the goal position is (goal_col, goal_row) with the map, map_input
		double h(int cur_col, int cur_row, int goal_col, int goal_row, const map_maker_ns::grid_map &map_input)
		{
			int i, d_col = abs(cur_col - goal_col), d_row = abs(cur_row - goal_row);
			double alpha = 0, beta = 0, e_d, m_d, res;
			for (i = 0; i < 12; i++)
			{
				int next_col = cur_col + (-2 + rand() % 5);
				int next_row = cur_row + (-2 + rand() % 5);
				if (map_input.check_available(next_col, next_row))
				{
					char bit_read = map_input.read_bit(next_col, next_row);
					if (bit_read == map_maker_ns::UNBLOCKED || bit_read == map_maker_ns::UNBLOCKED_HIGHWAY)
						alpha = alpha + 1;
					if (bit_read == map_maker_ns::UNBLOCKED_HIGHWAY || bit_read == map_maker_ns::HARD_HIGHWAY)
						beta = beta + 1;
				}
			}
			alpha = alpha / 12.0;
			beta = beta / 12.0;
			e_d = sqrt(d_col * d_col + d_row * d_row);
			m_d = d_col + d_row;
			res = (1 - beta) * e_d * (alpha + 2 * (1 - alpha)) + 0.25 * beta * m_d * (alpha + 2 * (1 - alpha));
			return res;
		}
	};
}

#endif
