#include "abstract_solver.h"

namespace solver_ns
{

	// build the path from (start_col, start_row) to (goal_col, goal_row) according to trace back, store the result to output
	void abstract_solver::build_result(int start_col, int start_row, int goal_col, int goal_row, map_maker_ns::result_path &output)
	{
		int cur_col = goal_col, cur_row = goal_row, next_col, next_row;

		output.list_row.clear();
		output.list_col.clear();
		
		while (1)
		{
			output.list_row.push_back(cur_row);
			output.list_col.push_back(cur_col);
			if (cur_col == start_col && cur_row == start_row)
				break;
			read_trace_back(cur_col, cur_row, next_col, next_row);
			cur_col = next_col;
			cur_row = next_row;
		}
	}


	// the interface for solving the problem where the input map is map_input, the start point is (start_col, start_row) and the goal point is (goal_col, goal_row)
	// return true when a path exist, otherwise return false
	// if a path exists, path_output stores the output path and expanded_nodes store the number of expanded nodes
	bool abstract_solver::solve(int start_col, int start_row, int goal_col, int goal_row, const map_maker_ns::grid_map &map_input, map_maker_ns::result_path &path_output, int &expanded_nodes)
	{
		int cur_col, cur_row, next_col, next_row, i, map_size_col, map_size_row;
		double cur_f, cur_g, next_f, next_g, edge_cost, exist_f, exist_g;

		// map_input.get_size(map_size_col, map_size_row);
		map_size_col = map_input.get_col_size();
		map_size_row = map_input.get_row_size();
		init(map_input, map_size_col, map_size_row);
		expanded_nodes = 0;
		fringe->insert(start_col, start_row, f(0, start_col, start_row, goal_col, goal_row, map_input), 0);
		set_trace_back(start_col, start_row, start_col, start_row);

		while (fringe->pop(cur_col, cur_row, cur_f, cur_g))
		{
			if (cur_col == goal_col && cur_row == goal_row)
			{
				build_result(start_col, start_row, goal_col, goal_row, path_output);
				return true;
			}
			closed_list->insert(cur_col, cur_row);
			expanded_nodes++;
			for (i = 0; i < map_maker_ns::MAX_DIR; i++)
			{
				next_col = cur_col + map_maker_ns::DELTA_COL[i];
				next_row = cur_row + map_maker_ns::DELTA_ROW[i];
				if (map_input.check_available(next_col, next_row))
				{
					edge_cost = map_maker_ns::result_path::moving_cost(cur_col, cur_row, next_col, next_row, map_input);
					if (!closed_list->exist(next_col, next_row))
					{
						next_g = cur_g + edge_cost;
						next_f = f(next_g, next_col, next_row, goal_col, goal_row, map_input);
						if (!fringe->get_value(next_col, next_row, exist_f, exist_g))
						{
							fringe->insert(next_col, next_row, next_f, next_g);
							set_trace_back(next_col, next_row, cur_col, cur_row);
						}
						else if (fringe->better(next_f, next_g, exist_f, exist_g))
						{
							fringe->update(next_col, next_row, next_f, next_g);
							set_trace_back(next_col, next_row, cur_col, cur_row);
						}
					}
				}
			}
		}
		
		return false;
	}

}
