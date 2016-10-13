#ifndef SEQUENTIAL_A_STAR_H
#define SEQUENTIAL_A_STAR_H

#include "abstract_solver.h"

namespace solver_ns
{
	class sequential_a_star: public abstract_solver
	{
		private:
			int n_heuristic;
			heuristic_ns::abstract_heuristic **heuristic_list;
			util_ns::my_heap **fringe_set;
			util_ns::my_hash **closed_set, *expanded_set;
			int **trace_back_col_set, **trace_back_row_set;
			double **f_set, **g_set, **h_set;
			double w1, w2;

			double f(double g, int cur_col, int cur_row, int goal_col, int goal_row, const map_maker_ns::grid_map &map_input)
			{
				return 0;
			}

			// return the f-value for the postion (cur_col, cur_row) while the goal is the (goal_col, goal_row) and the input map is map_input
			double f(double g, int cur_col, int cur_row, int goal_col, int goal_row, const map_maker_ns::grid_map &map_input, int heuristic_index)
			{
				return g + w1 * (heuristic_list[heuristic_index])->h(cur_col, cur_row, goal_col, goal_row, map_input);
			}

			void init_seq(const map_maker_ns::grid_map &map_input, int size_of_columns, int size_of_rows);
	
			// return 1d index for 2d position
			int index_2dto1d(int position_col, int position_row)
			{
				return position_row * n_columns + position_col;
			}

			// read the trace back value
			void read_trace_back(int index_heuristic, int position_col, int position_row, int &trace_column, int &trace_row)
			{
				trace_column = trace_back_col_set[index_heuristic][index_2dto1d(position_col, position_row)];
				trace_row = trace_back_row_set[index_heuristic][index_2dto1d(position_col, position_row)];
			}

			// set the trace back value
			void set_trace_back(int index_heuristic, int position_col, int position_row, int trace_column, int trace_row)
			{
				trace_back_col_set[index_heuristic][index_2dto1d(position_col, position_row)] = trace_column;
				trace_back_row_set[index_heuristic][index_2dto1d(position_col, position_row)] = trace_row;
			}

			void set_f_set(int index_heuristic, int position_col, int position_row, double f_input)
			{
				f_set[index_heuristic][index_2dto1d(position_col, position_row)] = f_input;
			}

			double get_f_set(int index_heuristic, int position_col, int position_row)
			{
				return f_set[index_heuristic][index_2dto1d(position_col, position_row)];
			}
		
			void set_g_set(int index_heuristic, int position_col, int position_row, double g_input)
			{
				g_set[index_heuristic][index_2dto1d(position_col, position_row)] = g_input;
			}

			double get_g_set(int index_heuristic, int position_col, int position_row)
			{
				return g_set[index_heuristic][index_2dto1d(position_col, position_row)];
			}

			void set_h_set(int index_heuristic, int position_col, int position_row, double h_input)
			{
				h_set[index_heuristic][index_2dto1d(position_col, position_row)] = h_input;
			}

			double get_h_set(int index_heuristic, int position_col, int position_row)
			{
				return h_set[index_heuristic][index_2dto1d(position_col, position_row)];
			}

			void expand_state(int index_heuristic, int position_col, int position_row, int goal_col, int goal_row, const map_maker_ns::grid_map &map_input);

			// build the path from (start_col, start_row) to (goal_col, goal_row) according to trace back for heuristic index_heuristic , store the result to output
			void build_result(int index_heuristic, int start_col, int start_row, int goal_col, int goal_row, map_maker_ns::result_path &output);

		public:
			sequential_a_star(int heuristic_size_input, double w1_input, double w2_input) : abstract_solver(NULL)
			{
				w1 = w1_input;	
				w2 = w2_input;
				heuristic_list = new heuristic_ns::abstract_heuristic*[1 + heuristic_size_input];
				n_heuristic = heuristic_size_input;
				fringe_set = NULL;
				closed_set = NULL;
				trace_back_col_set = NULL;
				trace_back_row_set = NULL;
				f_set = NULL;
				g_set = NULL;
				h_set = NULL;
				expanded_set = NULL;
			}

			void clear_sequential();

			~sequential_a_star()
			{
				clear_sequential();
				delete[] heuristic_list;
				heuristic_list = NULL;
			}

			void set_w1(double w1_input)
			{
				w1 = w1_input;
			}

			void set_w2(double w2_input)
			{
				w2 = w2_input;
			}

			void set_heuristic(int index_heuristic, heuristic_ns::abstract_heuristic *heuristic_input)
			{
				heuristic_list[index_heuristic] = heuristic_input;
			}

			// the interface for solving the problem where the input map is map_input, the start point is (start_col, start_row) and the goal point is (goal_col, goal_row)
			// return true when a path exist, otherwise return false
			// if a path exists, path_output stores the output path and 
			// expanded_nodes store the number of expanded nodes (without re-count), and overall_expanded store number of expanded nodes (with re-count)
			bool solve(int start_col, int start_row, int goal_col, int goal_row, const map_maker_ns::grid_map &map_input, map_maker_ns::result_path &path_output, int &expanded_nodes, int &overall_expanded);

			

	};
}

#endif
