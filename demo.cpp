#include "map_maker.h"
#include "uniform_cost_search.h"
#include "a_star.h"
#include "w_a_star.h"
#include "sequential_a_star.h"
#include "integrated_a_star.h"
#include "h1.h"
#include "h2.h"
#include "h3.h"
#include "h4.h"
#include "h5.h"
#include "demo_ui.h"

int main()
{
	map_maker_ns::map_maker map_input;
	map_maker_ns::result_path path_output;
	bool has_map = false, has_result = false;
	heuristic_ns::abstract_heuristic *h0_h = new heuristic_ns::abstract_heuristic();
	heuristic_ns::h1 *h1_h = new heuristic_ns::h1();
	heuristic_ns::h2 *h2_h = new heuristic_ns::h2();
	heuristic_ns::h3 *h3_h = new heuristic_ns::h3();
	heuristic_ns::h4 *h4_h = new heuristic_ns::h4();
	heuristic_ns::h5 *h5_h = new heuristic_ns::h5();
	
	while (1)
	{
		int input_number;

		input_number = demo_ui_ns::main_menu();
		if (input_number == 6)
			break;

		if (input_number == 1) // Create new map
		{
			has_map = true;
			has_result = false;
			map_input.map_build();
			cv::namedWindow("map");
			cv::Mat map_img = map_input.show_map_img();
			cv::imshow("map", map_img);
			cv::waitKey();
			cv::destroyAllWindows();
		}
		else if (input_number == 2) // Read map from file
		{
			char file_name[100];
			printf("file path: ");
			scanf("%s", file_name);
			has_map = true;
			has_result = false;
			map_input.read_text(file_name);
			cv::namedWindow("map");
			cv::Mat map_img = map_input.show_map_img();
			cv::imshow("map", map_img);
			cv::waitKey();
			cv::destroyAllWindows();
		}
		else if (input_number == 3) // Execute A* (WA* etc)
		{
			if (!has_map)
			{
				printf("No available map!\n");
				continue;
			}
			double w;
			int alg, h_input;
			int start_col, start_row, goal_col, goal_row, expanded;
			solver_ns::abstract_solver *solver = NULL;

			start_col = map_input.map_output.get_col_start();
			start_row = map_input.map_output.get_row_start();
			goal_col = map_input.map_output.get_col_goal();
			goal_row = map_input.map_output.get_row_goal();

			alg = demo_ui_ns::choose_algorithm();
			if (alg == 1)
				solver = new solver_ns::uniform_cost_search(NULL);
			else if (alg == 2)
				solver = new solver_ns::a_star(NULL);
			else if (alg == 3)
			{
				printf("weight: ");
				scanf("%lf", &w);
				solver = new solver_ns::w_a_star(NULL, w);
			}
	
			if (alg <= 3)
			{
				h_input = demo_ui_ns::choose_heuristic();
				if (h_input == 1)
					solver->set_heuristic(h1_h);
				else if (h_input == 2)
					solver->set_heuristic(h2_h);
				else if (h_input == 3)
					solver->set_heuristic(h3_h);
				else if (h_input == 4)
					solver->set_heuristic(h4_h);
				else if (h_input == 5)
					solver->set_heuristic(h5_h);
				else
					solver->set_heuristic(h0_h);
	
				if (solver->solve(start_col, start_row, goal_col, goal_row, map_input.map_output, path_output, expanded))
				{
					printf("cost for path = %f, expanded nodes = %d\n", path_output.compute_cost(map_input.map_output), expanded);
					has_result = true;
					solver->store_table("f_g_h.txt", h_input);
					cv::Mat map_result_img = map_input.show_result(map_input.map_output, path_output);
					cv::namedWindow("result");
					cvSetMouseCallback("result", map_input.mouse_event, NULL);
					cv::imshow("result", map_result_img);

					cv::waitKey();
					remove("f_g_h.txt");
					cv::destroyAllWindows();
				}
				else
					printf("no path!\n");
				delete solver;
				solver = NULL;
			}
			else
			{
				bool has_path;
				double w1, w2;
				int n_heuristic, admissiable, overall_expanded;
				std::vector<int> list_heuristic;
				demo_ui_ns::setting_for_seq_integ(w1, w2, n_heuristic, admissiable, list_heuristic);

				if (alg == 4) // sequential
				{
					solver_ns::sequential_a_star solver(n_heuristic, w1, w2);

					if (admissiable == 0)
						solver.set_heuristic(0, h0_h);
					else if (admissiable == 1)
						solver.set_heuristic(0, h1_h);
					else if (admissiable == 2)
						solver.set_heuristic(0, h2_h);
					else if (admissiable == 3)
						solver.set_heuristic(0, h3_h);
					else if (admissiable == 4)
						solver.set_heuristic(0, h4_h);
					else if (admissiable == 5)
						solver.set_heuristic(0, h5_h);
					for (int i = 0; i < list_heuristic.size(); i++)
						if (list_heuristic[i] == 0)
							solver.set_heuristic(i + 1, h0_h);
						else if (list_heuristic[i] == 1)
							solver.set_heuristic(i + 1, h1_h);
						else if (list_heuristic[i] == 2)
							solver.set_heuristic(i + 1, h2_h);
						else if (list_heuristic[i] == 3)
							solver.set_heuristic(i + 1, h3_h);
						else if (list_heuristic[i] == 4)
							solver.set_heuristic(i + 1, h4_h);
						else if (list_heuristic[i] == 5)
							solver.set_heuristic(i + 1, h5_h);
			
					has_path = solver.solve(start_col, start_row, goal_col, goal_row, map_input.map_output, path_output, expanded, overall_expanded);
					if (has_path)
						solver.store_table("f_g_h.txt", admissiable, list_heuristic);
				}
				else if (alg == 5) // integrated
				{
					solver_ns::integrated_a_star solver(n_heuristic, w1, w2);

					if (admissiable == 0)
						solver.set_heuristic(0, h0_h);
					else if (admissiable == 1)
						solver.set_heuristic(0, h1_h);
					else if (admissiable == 2)
						solver.set_heuristic(0, h2_h);
					else if (admissiable == 3)
						solver.set_heuristic(0, h3_h);
					else if (admissiable == 4)
						solver.set_heuristic(0, h4_h);
					else if (admissiable == 5)
						solver.set_heuristic(0, h5_h);
					for (int i = 0; i < list_heuristic.size(); i++)
						if (list_heuristic[i] == 0)
							solver.set_heuristic(i + 1, h0_h);
						else if (list_heuristic[i] == 1)
							solver.set_heuristic(i + 1, h1_h);
						else if (list_heuristic[i] == 2)
							solver.set_heuristic(i + 1, h2_h);
						else if (list_heuristic[i] == 3)
							solver.set_heuristic(i + 1, h3_h);
						else if (list_heuristic[i] == 4)
							solver.set_heuristic(i + 1, h4_h);
						else if (list_heuristic[i] == 5)
							solver.set_heuristic(i + 1, h5_h);
			
					has_path = solver.solve(start_col, start_row, goal_col, goal_row, map_input.map_output, path_output, expanded, overall_expanded);
					if (has_path)
						solver.store_table("f_g_h.txt", admissiable, list_heuristic);
				}

				if (has_path)
				{
					printf("cost for path = %f, expanded nodes = %d, overall expanded nodes = %d\n", path_output.compute_cost(map_input.map_output), expanded, overall_expanded);
					has_result = true;
					cv::Mat map_result_img = map_input.show_result(map_input.map_output, path_output);
					cv::namedWindow("result");
					cvSetMouseCallback("result", map_input.mouse_event, NULL);
					cv::imshow("result", map_result_img);

					cv::waitKey();
					remove("f_g_h.txt");
					cv::destroyAllWindows();
				}
				else
					printf("no path!\n");
			}
		}
		else if (input_number == 4) // Save result in file
		{
			if (!has_result)
			{
				printf("no available result!\n");
				continue;
			}
			char file_name[100];
			printf("file path: ");
			scanf("%s", file_name);
			cv::Mat result_img = map_input.show_result(map_input.map_output, path_output);
			cv::imwrite(file_name, result_img);
		}
		else if (input_number == 5) // Save map in file
		{
			if (!has_map)
			{
				printf("no available path!\n");
				continue;
			}
			char file_name[100];
			printf("file path: ");
			scanf("%s", file_name);
			map_input.write_img_to_disk(file_name);
		}
	}

	delete h0_h;
	h0_h = NULL;
	delete h1_h;
	h1_h = NULL;
	delete h2_h;
	h2_h = NULL;
	delete h3_h;
	h3_h = NULL;
	delete h4_h;
	h4_h = NULL;
	delete h5_h;
	h5_h = NULL;

	return 0;
}




