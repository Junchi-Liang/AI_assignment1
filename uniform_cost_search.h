#ifndef UNIFORM_COST_SEARCH_H
#define UNIFORM_COST_SEARCH_H

#include "abstract_solver.h"

namespace solver_ns
{
	class uniform_cost_search: public abstract_solver
	{
		protected:
			// return the f-value for the postion (cur_col, cur_row) while the goal is the (goal_col, goal_row) and the input map is map_input
			virtual double f(double g, int cur_col, int cur_row, int goal_col, int goal_row, const map_maker_ns::grid_map &map_input)
			{
				return g;
			}

		public:
			uniform_cost_search(heuristic_ns::abstract_heuristic *heuristic_input) : abstract_solver(heuristic_input) {}
	};
}

#endif
