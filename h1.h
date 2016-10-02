#ifndef H1_H
#define H1_H

#include "abstract_heuristic.h"

namespace heuristic_ns
{
	class h1: public abstract_heuristic
	{
		public:
		                      // return the h-value for the positon (cur_col, cur_row) while the goal position is (goal_col, goal_row) with the map, map_input
                        double h(int cur_col, int cur_row, int goal_col, int goal_row, const map_maker_ns::grid_map &map_input)
			{
				// debug
				printf("debug: h1\n");
				// debug end
				bool first = true;
				double min_cost;
				int i;

				for (i = 0; i < map_maker_ns::MAX_DIR; i++)
				{
					int next_col = cur_col + map_maker_ns::DELTA_COL[i], next_row = cur_row + map_maker_ns::DELTA_ROW[i];
					if (map_input.check_available(next_col, next_row))
					{
						double edge_cost = map_maker_ns::result_path::moving_cost(cur_col, cur_row, next_col, next_row, map_input);
						if (first || edge_cost < min_cost)
							first = false, min_cost = edge_cost;
					}
				}
				if (first)
				{
					min_cost = 100 * (map_input.get_col_size() * map_input.get_col_size() + map_input.get_row_size() * map_input.get_row_size());
					return min_cost;
				}
				double dis = (cur_col - goal_col) * (cur_col - goal_col) + (cur_row - goal_row) * (cur_row - goal_row);
				return dis * min_cost;
			}
	};
}

#endif
