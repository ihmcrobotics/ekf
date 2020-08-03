/*
 * NativeFilterMatrixOps.cpp
 *
 *  Created on: Sep 5, 2018
 *      Author: Georg Wiedebach
 */

#include <jni.h>
#include <Eigen/Dense>
#include "us_ihmc_ekf_filter_NativeFilterMatrixOpsWrapper.h"

using Eigen::MatrixXd;

typedef Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> JMatrixMap;

JNIEXPORT void JNICALL Java_us_ihmc_ekf_filter_NativeFilterMatrixOpsWrapper_computeABAt(JNIEnv *env, jobject thisObj, jdoubleArray result,
      jdoubleArray aData, jdoubleArray bData, jint n, jint m)
{
   jdouble *aDataArray = (jdouble*) env->GetPrimitiveArrayCritical(aData, NULL);
   jdouble *bDataArray = (jdouble*) env->GetPrimitiveArrayCritical(bData, NULL);
   jdouble *resultDataArray = (jdouble*) env->GetPrimitiveArrayCritical(result, NULL);

   JMatrixMap A(aDataArray, n, m);
   JMatrixMap B(bDataArray, m, m);
   JMatrixMap ABAt(resultDataArray, n, n);

   ABAt.noalias() = A * B * A.transpose();

   env->ReleasePrimitiveArrayCritical(aData, aDataArray, 0);
   env->ReleasePrimitiveArrayCritical(bData, bDataArray, 0);
   env->ReleasePrimitiveArrayCritical(result, resultDataArray, 0);
}

JNIEXPORT void JNICALL Java_us_ihmc_ekf_filter_NativeFilterMatrixOpsWrapper_predictErrorCovariance
  (JNIEnv *env, jobject thisObj, jdoubleArray result, jdoubleArray fData, jdoubleArray pData, jdoubleArray qData, jint n)
{
   jdouble *fDataArray = (jdouble*) env->GetPrimitiveArrayCritical(fData, NULL);
   jdouble *pDataArray = (jdouble*) env->GetPrimitiveArrayCritical(pData, NULL);
   jdouble *qDataArray = (jdouble*) env->GetPrimitiveArrayCritical(qData, NULL);
   jdouble *resultDataArray = (jdouble*) env->GetPrimitiveArrayCritical(result, NULL);

   JMatrixMap F(fDataArray, n, n);
   JMatrixMap P(pDataArray, n, n);
   JMatrixMap Q(qDataArray, n, n);
   JMatrixMap errorCovariance(resultDataArray, n, n);

   MatrixXd Qdiag = Q.diagonal().asDiagonal();
   errorCovariance.noalias() = F * P.selfadjointView<Eigen::Upper>() * F.transpose() + Qdiag;

   env->ReleasePrimitiveArrayCritical(fData, fDataArray, 0);
   env->ReleasePrimitiveArrayCritical(pData, pDataArray, 0);
   env->ReleasePrimitiveArrayCritical(qData, qDataArray, 0);
   env->ReleasePrimitiveArrayCritical(result, resultDataArray, 0);
}

JNIEXPORT void JNICALL Java_us_ihmc_ekf_filter_NativeFilterMatrixOpsWrapper_updateErrorCovariance
  (JNIEnv *env, jobject thisObj, jdoubleArray result, jdoubleArray kData, jdoubleArray hData, jdoubleArray pData, jint n, jint m)
{
   jdouble *kDataArray = (jdouble*) env->GetPrimitiveArrayCritical(kData, NULL);
   jdouble *hDataArray = (jdouble*) env->GetPrimitiveArrayCritical(hData, NULL);
   jdouble *pDataArray = (jdouble*) env->GetPrimitiveArrayCritical(pData, NULL);
   jdouble *resultDataArray = (jdouble*) env->GetPrimitiveArrayCritical(result, NULL);

   JMatrixMap K(kDataArray, m, n);
   JMatrixMap H(hDataArray, n, m);
   JMatrixMap P(pDataArray, m, m);
   JMatrixMap errorCovariance(resultDataArray, m, m);

   errorCovariance.noalias() = (MatrixXd::Identity(m, m) - K * H) * P.selfadjointView<Eigen::Upper>();

   env->ReleasePrimitiveArrayCritical(kData, kDataArray, 0);
   env->ReleasePrimitiveArrayCritical(hData, hDataArray, 0);
   env->ReleasePrimitiveArrayCritical(pData, pDataArray, 0);
   env->ReleasePrimitiveArrayCritical(result, resultDataArray, 0);
}

JNIEXPORT void JNICALL Java_us_ihmc_ekf_filter_NativeFilterMatrixOpsWrapper_computeKalmanGain
  (JNIEnv *env, jobject thisObj, jdoubleArray result, jdoubleArray pData, jdoubleArray hData, jdoubleArray rData, jint n, jint m)
{
   jdouble *pDataArray = (jdouble*) env->GetPrimitiveArrayCritical(pData, NULL);
   jdouble *hDataArray = (jdouble*) env->GetPrimitiveArrayCritical(hData, NULL);
   jdouble *rDataArray = (jdouble*) env->GetPrimitiveArrayCritical(rData, NULL);
   jdouble *resultDataArray = (jdouble*) env->GetPrimitiveArrayCritical(result, NULL);

   JMatrixMap P(pDataArray, m, m);
   JMatrixMap H(hDataArray, n, m);
   JMatrixMap R(rDataArray, n, n);
   JMatrixMap gain(resultDataArray, m, n);

   MatrixXd PHt = P.selfadjointView<Eigen::Upper>() * H.transpose();
   MatrixXd Rdiag = R.diagonal().asDiagonal();
   MatrixXd toInvert = H * PHt + Rdiag;
   gain.noalias() = PHt * toInvert.inverse();

   env->ReleasePrimitiveArrayCritical(pData, pDataArray, 0);
   env->ReleasePrimitiveArrayCritical(hData, hDataArray, 0);
   env->ReleasePrimitiveArrayCritical(rData, rDataArray, 0);
   env->ReleasePrimitiveArrayCritical(result, resultDataArray, 0);
}

JNIEXPORT void JNICALL Java_us_ihmc_ekf_filter_NativeFilterMatrixOpsWrapper_updateState
  (JNIEnv *env, jobject thisObj, jdoubleArray result, jdoubleArray xData, jdoubleArray kData, jdoubleArray rData, jint n, jint m)
{
   jdouble *xDataArray = (jdouble*) env->GetPrimitiveArrayCritical(xData, NULL);
   jdouble *kDataArray = (jdouble*) env->GetPrimitiveArrayCritical(kData, NULL);
   jdouble *rDataArray = (jdouble*) env->GetPrimitiveArrayCritical(rData, NULL);
   jdouble *resultDataArray = (jdouble*) env->GetPrimitiveArrayCritical(result, NULL);

   JMatrixMap x(xDataArray, n, 1);
   JMatrixMap K(kDataArray, n, m);
   JMatrixMap r(rDataArray, m, 1);
   JMatrixMap state(resultDataArray, n, 1);

   state.noalias() = x + K * r;

   env->ReleasePrimitiveArrayCritical(xData, xDataArray, 0);
   env->ReleasePrimitiveArrayCritical(kData, kDataArray, 0);
   env->ReleasePrimitiveArrayCritical(rData, rDataArray, 0);
   env->ReleasePrimitiveArrayCritical(result, resultDataArray, 0);
}
