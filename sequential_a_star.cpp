#include "sequential_a_star.h"

namespace solver_ns
{

	// build the path from (start_col, start_row) to (goal_col, goal_row) according to trace back for heuristic index_heuristic , store the result to output
	void sequential_a_star::build_result(int index_heuristic, int start_col, int start_row, int goal_col, int goal_row, map_maker_ns::result_path &output)
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
			read_trace_back(index_heuristic, cur_col, cur_row, next_col, next_row);
			cur_col = next_col;
			cur_row = next_row;
		}
	}

	void sequential_a_star::clear_sequential()
	{
		int i;
		if (fringe_set != NULL)
		{
			for (i = 0; i <= n_heuristic; i++)
			{
				delete fringe_set[i];
				fringe_set[i] = NULL;
			}
			delete[] fringe_set;
		}
		fringe_set = NULL;
		if (closed_set != NULL)
		{
			for (i = 0; i <= n_heuristic; i++)
			{
				delete closed_set[i];
				closed_set[i] = NULL;
			}
			delete[] closed_set;
		}
		closed_set = NULL;
		if (trace_back_col_set != NULL)
		{
			for (i = 0; i <= n_heuristic; i++)
			{
				delete trace_back_col_set[i];
				trace_back_col_set[i] = NULL;
			}
			delete[] trace_back_col_set;
		}
		trace_back_col_set = NULL;
		if (trace_back_row_set != NULL)
		{
			for (i = 0; i <= n_heuristic; i++)
			{
				delete trace_back_row_set[i];
				trace_back_row_set[i] = NULL;
			}
			delete[] trace_back_row_set;
		}
		trace_back_row_set = NULL;
		if (f_set != NULL)
		{
			for (i = 0; i <= n_heuristic; i++)
			{
				delete f_set[i];
				f_set[i] = NULL;
			}
			delete[] f_set;
		}
		f_set = NULL;
		if (g_set != NULL)
		{
			for (i = 0; i <= n_heuristic; i++)
			{
				delete g_set[i];
				g_set[i] = NULL;
			}
			delete[] g_set;
		}	
		g_set = NULL;
		if (h_set != NULL)
		{
			for (i = 0; i <= n_heuristic; i++)
			{
				delete h_set[i];
				h_set[i] = NULL;
			}
			delete[] h_set;
		}
		h_set = NULL;
		if (expanded_set != NULL)
			delete expanded_set;
		expanded_set = NULL;
	}

	void sequential_a_star::init_seq(const map_maker_ns::grid_map &map_input, int size_of_columns, int size_of_rows)
	{
		int i, j, cell_size = size_of_columns * size_of_rows + 5;
		clear_sequential();
		n_columns = size_of_columns;
		m_rows = size_of_rows;
		fringe_set = new util_ns::my_heap*[1 + n_heuristic];
		closed_set = new util_ns::my_hash*[1 + n_heuristic];
		trace_back_col_set = new int*[1 + n_heuristic];
		trace_back_row_set = new int*[1 + n_heuristic];
		f_set = new double*[1 + n_heuristic];
		g_set = new double*[1 + n_heuristic];
		h_set = new double*[1 + n_heuristic];
		for (i = 0; i <= n_heuristic; i++)
		{
			fringe_set[i] = new util_ns::my_heap(size_of_columns, size_of_rows);
			closed_set[i] = new util_ns::my_hash(size_of_columns, size_of_rows);
			trace_back_col_set[i] = new int[cell_size];
			trace_back_row_set[i] = new int[cell_size];
			f_set[i] = new double[cell_size];
			g_set[i] = new double[cell_size];
			h_set[i] = new double[cell_size];
			for (j = 0; j < cell_size; j++)
			{
				trace_back_col_set[i][j] = -1;
				trace_back_row_set[i][j] = -1;
				f_set[i][j] = g_set[i][j] = h_set[i][j] = -1;
			}
		}
		expanded_set = new util_ns::my_hash(size_of_columns, size_of_rows);
	}

	void sequential_a_star::expand_state(int index_heuristic, int position_col, int position_row, int goal_col, int goal_row, const map_maker_ns::grid_map &map_input)
	{
		int i;
		double f_cur, g_cur;

		(fringe_set[index_heuristic])->get_value(position_col, position_row, f_cur, g_cur);
		(fringe_set[index_heuristic])->remove(position_col, position_row);

		for (i = 0; i < map_maker_ns::MAX_DIR; i++)
		{
			int next_col = position_col + map_maker_ns::DELTA_COL[i];
			int next_row = position_row + map_maker_ns::DELTA_ROW[i];
			if (map_input.check_available(next_col, next_row))
			{
				double f_old, g_old;
				f_old = get_f_set(index_heuristic, next_col, next_row);
				g_old = get_g_set(index_heuristic, next_col, next_row);
				double edge_cost = map_maker_ns::result_path::moving_cost(position_col, position_row, next_col, next_row, map_input);
				double f_new, g_new = g_cur + edge_cost;
				f_new = f(g_new, next_col, next_row, goal_col, goal_row, map_input, index_heuristic);
				if (g_old < 0 || g_new < g_old)
				{
					set_trace_back(index_heuristic, next_col, next_row, position_col, position_row);
					set_g_set(index_heuristic, next_col, next_row, g_new);
					if (!(closed_set[index_heuristic])->exist(next_col, next_row))
					{
						if (!((fringe_set[index_heuristic])->exist(next_col, next_row)))
							(fringe_set[index_heuristic])->insert(next_col, next_row, f_new, g_new);
						else
							(fringe_set[index_heuristic])->update(next_col, next_row, f_new, g_new);
					}
				}
			}
		}
	}

	// the interface for solving the problem where the input map is map_input, the start point is (start_col, start_row) and the goal point is (goal_col, goal_row)
	// return true when a path exist, otherwise return false
	// if a path exists, path_output stores the output path and
	// expanded_nodes store the number of expanded nodes (without re-count), and overall_expanded store number of expanded nodes (with re-count)
	bool sequential_a_star::solve(int start_col, int start_row, int goal_col, int goal_row, const map_maker_ns::grid_map &map_input, map_maker_ns::result_path &path_output, int &expanded_nodes, int &overall_expanded)
	{
		int map_size_col, map_size_row, i, cur_col_0, cur_row_0, cur_col_i, cur_row_i;
		double f0, g0, fi, gi;
		bool has_element_0 = true;

		map_size_col = map_input.get_col_size();
		map_size_row = map_input.get_row_size();

		init_seq(map_input, map_size_col, map_size_row);
		expanded_nodes = 0;
		overall_expanded = 0;

		for (i = 0; i <= n_heuristic; i++)
		{
			g0 = 0;
			f0 = f(g0, start_col, start_row, goal_col, goal_row, map_input, i);
			(fringe_set[i])->insert(start_col, start_row, f0, g0);
		}


		while (has_element_0)
		{
			for (i = 1; i <= n_heuristic && has_element_0; i++)
			{
				has_element_0 = (fringe_set[0])->get_best(cur_col_0, cur_row_0, f0, g0);
				if (!has_element_0)
					break;
				bool has_element = (fringe_set[i])->get_best(cur_col_i, cur_row_i, fi, gi);

				if (has_element && fi <= w2 * f0)
				{
					set_f_set(i, cur_col_i, cur_row_i, fi);
					set_g_set(i, cur_col_i, cur_row_i, gi);
					set_h_set(i, cur_col_i, cur_row_i, (fi - gi) / w1);
					if (cur_col_i == goal_col && cur_row_i == goal_row)
					{
						build_result(i, start_col, start_row, goal_col, goal_row, path_output);
						return true;
					}
					overall_expanded++;
					if (!expanded_set->exist(cur_col_i, cur_row_i))
					{
						expanded_nodes++;
						expanded_set->insert(cur_col_i, cur_row_i);
					}
					expand_state(i, cur_col_i, cur_row_i, goal_col, goal_row, map_input);
					(closed_set[i])->insert(cur_col_i, cur_row_i);
				}
				else
				{
					set_f_set(0, cur_col_0, cur_row_0, f0);
					set_g_set(0, cur_col_0, cur_row_0, g0);
					set_h_set(0, cur_col_0, cur_row_0, (f0 - g0) / w1);

					if (cur_col_0 == goal_col && cur_row_0 == goal_row)
					{
						build_result(0, start_col, start_row, goal_col, goal_row, path_output);
						return true;
					}
					overall_expanded++;
					if (!expanded_set->exist(cur_col_0, cur_row_0))
					{
						expanded_nodes++;
						expanded_set->insert(cur_col_0, cur_row_0);
					}
					expand_state(0, cur_col_0, cur_row_0, goal_col, goal_row, map_input);
					(closed_set[0])->insert(cur_col_0, cur_row_0);
				}
			}
		}
		
		return false;
	}

}





