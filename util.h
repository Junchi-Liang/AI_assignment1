#ifndef UTIL_H
#define UTIL_H

#include <iostream>
#include <cstdio>
#include <cstdlib>

namespace util_ns
{

	// our implement for the heap
	class my_heap
	{
		my_heap(int size_of_columns, int size_of_rows);
		~my_heap();
		private:
			int *heap_col, *heap_row, capacity, size;
			double *heap_f;
			int n_columns, m_rows;
			int **heap_index;
		public:
			bool insert(int position_col, int position_row, double f); // insert the cell (position_col, position_row) with f-value f to the heap
			bool remove(int position_col, int position_row); // remove the cell (position_col, position_row) from the heap
			bool pop(int &position_col, int position_row, double &f); // pop the root of the heap, return its information
			bool exist(int position_col, int position_row); // check if the cell (position_col, position_row) exists in the heap, return true if exists
	};

	// our implement for the hash used for the closed list
	class my_hash
	{
		my_hash(int size_of_columns, int size_of_rows);
		~my_hash();
		private:
			int n_columns, m_rows;
			bool *hash_bit;
		public:
			bool insert(int position_col, int position_row); // insert the cell (position_col, position_row) into the hash
			bool remove(int position_col, int position_row); // remove the cell (position_col, position_row) from the hash
			bool exist(int position_col, int position_row); // check if the cell (position_col, position_row) exists in the hash
	};
		
}

#endif
