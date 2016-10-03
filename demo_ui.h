#ifndef DEMO_UI_H
#define DEMO_UI_H

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace demo_ui_ns
{

	int main_menu()
	{
		int input_num;
		while (1)
		{
			printf("Please input your choice:\n1. Create new map\n2. Read map from file\n3. Execute A* (WA* etc)\n4. Save result in file\n5. Save map in file\n6. Exit\n\nEnter your choice(1-6): ");
			scanf("%d", &input_num);
			if (input_num >= 1 && input_num <= 6)
				break;
			printf("please input a number from 1 to 6!\n\n");
		}
		return input_num;
	}

	int choose_algorithm()
	{
		int input_num;
		while (1)
		{
			printf("Please input algorithm:\n1.Uniform Cost Search\n2.A*\n3.Weighted A*\n\nEnter your choice(1-3): ");
			scanf("%d", &input_num);
			if (input_num >= 1 && input_num <= 3)
				break;
			printf("please input a number from 1 to 3!\n\n");
		}
		return input_num;
	}

	int choose_heuristic()
	{
		int input_num;
		while (1)
		{
			printf("Please input heurisric:\n0.h0\n1.h1\n2.h2\n3.h3\n4.h4\n5.h5\n\nEnter your choice(0-5): ");
			scanf("%d", &input_num);
			if (input_num >= 0 && input_num <= 5)
				break;
			printf("please input a number from 0 to 5!\n\n");
		}
		return input_num;
	}

}

#endif
