//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// _coder_DSL_api.h
//
// Code generation for function 'DSL'
//

#ifndef _CODER_DSL_API_H
#define _CODER_DSL_API_H

// Include files
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void DSL(real_T z[22], real_T k[24], real_T param[4], real_T ref[15], real_T t,
         real_T F1[3], real_T F2[3], real_T xidot[4]);

void DSL_api(const mxArray *const prhs[5], int32_T nlhs,
             const mxArray *plhs[3]);

void DSL_atexit();

void DSL_initialize();

void DSL_terminate();

void DSL_xil_shutdown();

void DSL_xil_terminate();

#endif
// End of code generation (_coder_DSL_api.h)
