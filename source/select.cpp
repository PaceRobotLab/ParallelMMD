#define SWAP(a,b) temp=(a);(a)=(b);(b)=temp;
#include "select.h"


float median(float arr[], int n)
{
	int k = (int)(n/2);
	return select(k,n,arr);
}

float select(int k, int n, float arr[])
{
	int i,ir,j,l,mid;
	float a,temp;

	l=1;
	ir=n;
	for (;;) {
		if (ir <= l+1) {
			if (ir == l+1 && arr[ir] < arr[l]) {
				SWAP(arr[l],arr[ir])
			}
			return arr[k];
		} else {
			mid=(l+ir) >> 1;
			SWAP(arr[mid],arr[l+1])
			if (arr[l+1] > arr[ir]) {
				SWAP(arr[l+1],arr[ir])
			}
			if (arr[l] > arr[ir]) {
				SWAP(arr[l],arr[ir])
			}
			if (arr[l+1] > arr[l]) {
				SWAP(arr[l+1],arr[l])
			}
			i=l+1;
			j=ir;
			a=arr[l];
			for (;;) {
				do i++; while (arr[i] < a);
				do j--; while (arr[j] > a);
				if (j < i) break;
				SWAP(arr[i],arr[j])
			}
			arr[l]=arr[j];
			arr[j]=a;
			if (j >= k) ir=j-1;
			if (j <= k) l=i;
		}
	}
}

void sort20(int* array, int n)
{
	//sort array of uop to 20 elements
	int i,j,tmp;
	
	for(i=0;i<n-1;i++)
	for(j=i+1; j<n;j++)
		if(array[i]>array[j])
		{
			tmp = array[j];
			array[j] = array[i];
			array[i] = tmp;
		}
			
}




#undef SWAP
/* (C) Copr. 1986-92 Numerical Recipes Software n$<39$`95.){2p49J%#4. */
