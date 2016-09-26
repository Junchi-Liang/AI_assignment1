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
		private:
			int *heap_col, *heap_row, capacity, size;
			double *heap_f, *heap_g;
			int n_columns, m_rows;
			int *heap_index;
			bool swap(int a, int b); // swap node a and node b
			bool compare(int a, int b); // return true when node a is better than node b
		public:
			my_heap(int size_of_columns, int size_of_rows);
			~my_heap();
			bool insert(int position_col, int position_row, double f, double g); // insert the cell (position_col, position_row) with f-value as f and g-value as g to the heap
			bool remove(int position_col, int position_row); // remove the cell (position_col, position_row) from the heap
			bool pop(int &position_col, int &position_row, double &f, double &g); // pop the root of the heap, return its information
			int move(int node); // move the node in order to maintain the heap, return final postion of this node
			bool exist(int position_col, int position_row); // check if the cell (position_col, position_row) exists in the heap, return true if exists
			bool update(int position_col, int position_row, double f, double g); // update the cell (position_col, position_row) with f-value as f and g-value as g
	};

	// our implement for the hash used for the closed list
	class my_hash
	{
		private:
			int n_columns, m_rows;
			bool *hash_bit;
			int hash_index(int position_col, int position_row); // hash function
		public:
			my_hash(int size_of_columns, int size_of_rows);
			~my_hash();
			bool insert(int position_col, int position_row); // insert the cell (position_col, position_row) into the hash
			bool remove(int position_col, int position_row); // remove the cell (position_col, position_row) from the hash
			bool exist(int position_col, int position_row); // check if the cell (position_col, position_row) exists in the hash
	};
		
}

#endif
