//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// main.cpp
//
// Code generation for function 'main'
//

/*************************************************************************/
/* This automatically generated example C++ main file shows how to call  */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/

// Include files
#include "main.h"
#include "DSL.h"
#include "DSL_terminate.h"
#include "rt_nonfinite.h"

// Function Declarations
static void argInit_1x15_real_T(double result[15]);

static void argInit_1x4_real_T(double result[4]);

static void argInit_22x1_real_T(double result[22]);

static void argInit_6x4_real_T(double result[24]);

static double argInit_real_T();

// Function Definitions
static void argInit_1x15_real_T(double result[15])
{
  // Loop over the array to initialize each element.
  for (int idx1{0}; idx1 < 15; idx1++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx1] = argInit_real_T();
  }
}

static void argInit_1x4_real_T(double result[4])
{
  // Loop over the array to initialize each element.
  for (int idx1{0}; idx1 < 4; idx1++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx1] = argInit_real_T();
  }
}

static void argInit_22x1_real_T(double result[22])
{
  // Loop over the array to initialize each element.
  for (int idx0{0}; idx0 < 22; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real_T();
  }
}

static void argInit_6x4_real_T(double result[24])
{
  // Loop over the array to initialize each element.
  for (int i{0}; i < 24; i++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[i] = argInit_real_T();
  }
}

static double argInit_real_T()
{
  return 0.0;
}

int main(int, char **)
{
  // The initialize function is being called automatically from your entry-point
  // function. So, a call to initialize is not included here. Invoke the
  // entry-point functions.
  // You can call entry-point functions multiple times.
  main_DSL();
  // Terminate the application.
  // You do not need to do this more than one time.
  DSL_terminate();
  return 0;
}

void main_DSL()
{
  double dv1[24];
  double dv[22];
  double dv3[15];
  double dv2[4];
  double xidot[4];
  double F1[3];
  double F2[3];
  // Initialize function 'DSL' input arguments.
  // Initialize function input argument 'z'.
  // Initialize function input argument 'k'.
  // Initialize function input argument 'param'.
  // Initialize function input argument 'ref'.
  // Call the entry-point 'DSL'.
  argInit_22x1_real_T(dv);
  argInit_6x4_real_T(dv1);
  argInit_1x4_real_T(dv2);
  argInit_1x15_real_T(dv3);
  DSL(dv, dv1, dv2, dv3, argInit_real_T(), F1, F2, xidot);
}

// End of code generation (main.cpp)
