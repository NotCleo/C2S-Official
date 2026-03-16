#include <stdio.h>
#include <stdlib.h>
#include <math.h>

int compare_floats(const void* a, const void* b)
  {
      const float* row_a = (const float*)a;
      const float* row_b = (const float*)b;

      if (row_a[0] < row_b[0]) return -1;
      else if (row_a[0] > row_b[0]) return 1;
      else return 0;
  }

int main()
  {
      int k[10] = {1,2,3,5,7,9,10,15,20,22}; 
      
      float age[22] = {19,35,26,27,19,27,27,32,25,35,26,26,20,32,18,29,47,45,46,48,45,47}; 
      float salary[22] = {19000,20000,43000,57000,76000,58000,84000,150000,33000,65000,80000,52000,86000,18000,82000,80000,25000,26000,28000,29000,22000,49000}; 
      
      float max_sal = salary[0], min_sal = salary[0];
      float max_age = age[0], min_age = age[0];
      
      for(int i=1; i<22; i++){
        if(salary[i] > max_sal) max_sal = salary[i];
        if(salary[i] < min_sal) min_sal = salary[i];
        
        if(age[i] > max_age) max_age = age[i];
        if(age[i] < min_age) min_age = age[i];
      }
      
      for(int i=0; i<22; i++){
        salary[i] = (salary[i] - min_sal) / (max_sal - min_sal);  
        age[i] = (age[i] - min_age) / (max_age - min_age);
      }

      float labels[22] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1};
      float distance[22][2]; 
      
      float pred_age = 0.0; 
      float pred_salary = 0.0; 
      
      printf("Enter Age and Salary: ");
      scanf("%f %f", &pred_age, &pred_salary); 
      
      pred_salary = (pred_salary - min_sal) / (max_sal - min_sal);
      pred_age = (pred_age - min_age) / (max_age - min_age);
      
      for(int i = 0; i < 22; i++)
          {
             distance[i][0] = sqrt(pow(age[i]-pred_age, 2) + pow(salary[i]-pred_salary, 2));    
             distance[i][1] = labels[i]; 
          } 

      qsort(distance, 22, sizeof(distance[0]), compare_floats); 
      
      for(int h=0; h<10; h++){   
          printf("\nFor K value : %d\n", k[h]);
          
            int count_0 = 0; 
            int count_1 = 0;
            
            for(int g=0; g<k[h]; g++){
              if(distance[g][1] == 0.0) count_0++;
              if(distance[g][1] == 1.0) count_1++;
            }
            
            if(count_1 > count_0){              
              printf("Prediction: Customer buys the SUV\n");              
            }
            else{
              printf("Prediction: Customer does NOT buy the SUV\n"); 
            }
            printf("---");
        }

  }
