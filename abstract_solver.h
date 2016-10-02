#ifndef ABSTRACT_SOLVER_H
#define ABSTRACT_SOLVER_H

#include <iostream>
#include <cstdio>
#include <cstdlib>

#include "abstract_heuristic.h"
#include "util.h"

namespace solver_ns
{
	class abstract_solver
	{
		protected:
			heuristic_ns::abstract_heuristic *heuristic;
			util_ns::my_heap *fringe;
			util_ns::my_hash *closed_list;
			int *trace_back_col, *trace_back_row;
			int n_columns, m_rows;

			// return the f-value for the postion (cur_col, cur_row) while the goal is the (goal_col, goal_row) and the input map is map_input
			virtual double f(double g, int cur_col, int cur_row, int goal_col, int goal_row, const map_maker_ns::grid_map &map_input) = 0;

			// initialize with a map of size (size_of_columns * size_of_rows)
			void init(const map_maker_ns::grid_map &map_input, int size_of_columns, int size_of_rows)
			{
				n_columns = size_of_columns;
				m_rows = size_of_rows;
				if (fringe != NULL)
					delete fringe;
				if (closed_list != NULL)
					delete closed_list;
				if (trace_back_col != NULL)
					delete[] trace_back_col;
				if (trace_back_row != NULL)
					delete[] trace_back_row;
				fringe = new util_ns::my_heap(size_of_columns, size_of_rows);
				closed_list = new util_ns::my_hash(size_of_columns, size_of_rows);
				trace_back_col = new int[n_columns * m_rows + 5];
				trace_back_row = new int[n_columns * m_rows + 5];
			}

			// read the trace back value
			void read_trace_back(int position_col, int position_row, int &trace_column, int &trace_row)
			{
				trace_column = trace_back_col[position_row * n_columns + position_col];
				trace_row = trace_back_row[position_row * n_columns + position_col];
			}

			// set the trace back value
			void set_trace_back(int position_col, int position_row, int trace_column, int trace_row)
			{
				trace_back_col[position_row * n_columns + position_col] = trace_column;
				trace_back_row[position_row * n_columns + position_col] = trace_row;
			}

			// build the path from (start_col, start_row) to (goal_col, goal_row) according to trace back, store the result to output
			void build_result(int start_col, int start_row, int goal_col, int goal_row, map_maker_ns::result_path &output);
			
		public:
			abstract_solver(heuristic_ns::abstract_heuristic *heuristic_input)
			{
				fringe = NULL;
				closed_list = NULL;
				trace_back_col = NULL;
				trace_back_row = NULL;
				heuristic = heuristic_input;
			}
			~abstract_solver()
			{
				heuristic = NULL;
				if (fringe != NULL)
					delete fringe;
				fringe = NULL;
				if (closed_list != NULL)
					delete closed_list;
				closed_list = NULL;
				if (trace_back_col != NULL)
					delete[] trace_back_col;
				trace_back_col = NULL;
				if (trace_back_row != NULL)
					delete[] trace_back_row;
				trace_back_row = NULL;
			}
			void set_heuristic(heuristic_ns::abstract_heuristic *heuristic_input)
			{
				heuristic = heuristic_input;
			}
			
			// the interface for solving the problem where the input map is map_input, the start point is (start_col, start_row) and the goal point is (goal_col, goal_row)
			// return true when a path exist, otherwise return false
			// if a path exists, path_output stores the output path and expanded_nodes store the number of expanded nodes
			bool solve(int start_col, int start_row, int goal_col, int goal_row, const map_maker_ns::grid_map &map_input, map_maker_ns::result_path &path_output, int &expanded_nodes);
			
	};
};

#endif

