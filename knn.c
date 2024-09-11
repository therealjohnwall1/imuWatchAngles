#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <assert.h>

#define LINE_BUF_SIZE 10000
#define K_NEIGH 3
#define FILE_NAME "values.csv"
#define ROW_SIZE 41 

struct dataPoint {
    float euclidDist;
    int class;
};

float euclidDistance(float* f1, float* f2, int rowSize){
    float tEuclid = 0;
    for(int i = 1; i <  rowSize; i++) {
        tEuclid = pow(f1[i] - f2[i],2);
    }
    return sqrt(tEuclid);
}


float* parseStringtoArr(const char* str, int num_floats) {
    float* res = (float*) malloc(num_floats * sizeof(float));

    char temBuffer[LINE_BUF_SIZE];
    int tempDex = 0;
    int col= 0;
    int i = 0;

   while(str[i] != '\0') {
        if (str[i] != ',') {
            temBuffer[tempDex++] = str[i];
        }

        else {
            temBuffer[tempDex] = '\0';
            res[col] = strtof(temBuffer, NULL);
            tempDex = 0;
            col++;
        }
        i++;
    }

    if(tempDex != 0) {
        res[col++] = strtof(temBuffer, NULL);
    }

    assert(tempDex > 0);
    temBuffer[tempDex] = '\0';
    res[col] = strtof(temBuffer, NULL);
    assert(col == num_floats);
    return res;
}


float** fillTable (int* i) {
    int rows = 101 ;
    FILE* values = fopen(FILE_NAME, "r");
    float** table = (float**)malloc(rows * sizeof(float*));

    char buffer[LINE_BUF_SIZE];
    fgets(buffer, LINE_BUF_SIZE,values);

    while(fgets(buffer, LINE_BUF_SIZE, values)) {
        table[*i] = parseStringtoArr(buffer, ROW_SIZE);     
        *i+=1;
    }

    fclose(values);
    return table;
}

int compare(const void *euc1, const void *euc2) {
    return (((struct dataPoint*)euc1)->euclidDist - ((struct dataPoint*)euc2)->euclidDist);
}

int classify(float* entryPoint, float** values, int valuesSize) { 
    struct dataPoint points[valuesSize];
    for(int i = 0; i < valuesSize;i++) {
        points[i].euclidDist = euclidDistance(entryPoint, values[i], ROW_SIZE);
        points[i].class = values[i][3];
        printf("point: %d\n", points[i].class);
    }

    qsort(points, valuesSize, sizeof(struct dataPoint), compare);

    int class0 = 0, class1= 0, class2= 0;
    for(int i = 0; i < ROW_SIZE;i++) {
        int class = points[i].class;
        switch(class) {
            case 0:
                class0++;
                break;
            case 1:
                class1++;
                break;
            case 2:
                class2++;
                break;
        }
    }
    if (class0 >= class1 && class0 >= class2) {
        return 0;
    }
    else if (class1 >= class0 && class1 >= class2) {
        return 1;
    }
    else {
        return 2;
    }
}

int main() {
    int tSize = 0;
    float** vals = fillTable(&tSize);
    // int valuesSize = 100;
    float entry[ROW_SIZE] = {1.000000,0.000000,0.000000,0.000000,0.999146,-0.033480,0.010957,0.021592,0.986657,-0.133411,0.043813,0.082396,0.934415,-0.292924,0.095825,0.178553,0.799492,-0.497481,0.160356,0.295993,0.532224,-0.707404,0.222948,0.408180,0.108567,-0.838489,0.258856,0.467057,-0.416964,-0.776559,0.229986,0.412557,-0.870471,-0.432459,0.106955,0.209334,-0.979216,0.152615,-0.084097,-0.103787};
    int predClass = classify(entry, vals, tSize);
    printf("Predicted class: %d\n", predClass);

    return 0;
}
