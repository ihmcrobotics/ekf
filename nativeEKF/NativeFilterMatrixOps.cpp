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

JNIEXPORT void JNICALL Java_us_ihmc_ekf_filter_NativeFilterMatrixOpsWrapper_computeABAt(JNIEnv *env, jobject thisObj, jdoubleArray result,
		jdoubleArray aData, jdoubleArray bData, jint n, jint m)
{
	jdouble *aDataArray = env->GetDoubleArrayElements(aData, NULL);
	jdouble *bDataArray = env->GetDoubleArrayElements(bData, NULL);
	MatrixXd A = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(aDataArray, n, m);
	MatrixXd B = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(bDataArray, m, m);

	MatrixXd ABAt = A * B * A.transpose();

	jdouble *resultDataArray = new jdouble[n * n];
	Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(resultDataArray, n, n) = ABAt;
	env->SetDoubleArrayRegion(result, 0, n * n, resultDataArray);

	env->ReleaseDoubleArrayElements(aData, aDataArray, 0);
	env->ReleaseDoubleArrayElements(bData, bDataArray, 0);
	delete resultDataArray;
}

JNIEXPORT void JNICALL Java_us_ihmc_ekf_filter_NativeFilterMatrixOpsWrapper_predictErrorCovariance
  (JNIEnv *env, jobject thisObj, jdoubleArray result, jdoubleArray fData, jdoubleArray pData, jdoubleArray qData, jint n)
{
	jdouble *fDataArray = env->GetDoubleArrayElements(fData, NULL);
	jdouble *pDataArray = env->GetDoubleArrayElements(pData, NULL);
	jdouble *qDataArray = env->GetDoubleArrayElements(qData, NULL);
	MatrixXd F = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(fDataArray, n, n);
	MatrixXd P = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(pDataArray, n, n);
	MatrixXd Q = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(qDataArray, n, n);

	MatrixXd Qdiag = Q.diagonal().asDiagonal();
	MatrixXd errorCovariance = F * P.selfadjointView<Eigen::Upper>() * F.transpose() + Qdiag;

	jdouble *resultDataArray = new jdouble[n * n];
	Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(resultDataArray, n, n) = errorCovariance;
	env->SetDoubleArrayRegion(result, 0, n * n, resultDataArray);

	env->ReleaseDoubleArrayElements(fData, fDataArray, 0);
	env->ReleaseDoubleArrayElements(pData, pDataArray, 0);
	env->ReleaseDoubleArrayElements(qData, qDataArray, 0);
	delete resultDataArray;
}

JNIEXPORT void JNICALL Java_us_ihmc_ekf_filter_NativeFilterMatrixOpsWrapper_updateErrorCovariance
  (JNIEnv *env, jobject thisObj, jdoubleArray result, jdoubleArray kData, jdoubleArray hData, jdoubleArray pData, jint n, jint m)
{
	jdouble *kDataArray = env->GetDoubleArrayElements(kData, NULL);
	jdouble *hDataArray = env->GetDoubleArrayElements(hData, NULL);
	jdouble *pDataArray = env->GetDoubleArrayElements(pData, NULL);
	MatrixXd K = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(kDataArray, m, n);
	MatrixXd H = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(hDataArray, n, m);
	MatrixXd P = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(pDataArray, m, m);

	MatrixXd errorCovariance = (MatrixXd::Identity(m, m) - K * H) * P.selfadjointView<Eigen::Upper>();

	jdouble *resultDataArray = new jdouble[m * m];
	Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(resultDataArray, m, m) = errorCovariance;
	env->SetDoubleArrayRegion(result, 0, m * m, resultDataArray);

	env->ReleaseDoubleArrayElements(kData, kDataArray, 0);
	env->ReleaseDoubleArrayElements(hData, hDataArray, 0);
	env->ReleaseDoubleArrayElements(pData, pDataArray, 0);
	delete resultDataArray;
}

JNIEXPORT void JNICALL Java_us_ihmc_ekf_filter_NativeFilterMatrixOpsWrapper_computeKalmanGain
  (JNIEnv *env, jobject thisObj, jdoubleArray result, jdoubleArray pData, jdoubleArray hData, jdoubleArray rData, jint n, jint m)
{
	jdouble *pDataArray = env->GetDoubleArrayElements(pData, NULL);
	jdouble *hDataArray = env->GetDoubleArrayElements(hData, NULL);
	jdouble *rDataArray = env->GetDoubleArrayElements(rData, NULL);
	MatrixXd P = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(pDataArray, m, m);
	MatrixXd H = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(hDataArray, n, m);
	MatrixXd R = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(rDataArray, n, n);

	MatrixXd PHt = P.selfadjointView<Eigen::Upper>() * H.transpose();
	MatrixXd Rdiag = R.diagonal().asDiagonal();
	MatrixXd toInvert = H * PHt + Rdiag;
	MatrixXd kalmanGain = PHt * toInvert.inverse();

	jdouble *resultDataArray = new jdouble[m * n];
	Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(resultDataArray, m, n) = kalmanGain;
	env->SetDoubleArrayRegion(result, 0, m * n, resultDataArray);

	env->ReleaseDoubleArrayElements(pData, pDataArray, 0);
	env->ReleaseDoubleArrayElements(hData, hDataArray, 0);
	env->ReleaseDoubleArrayElements(rData, rDataArray, 0);
	delete resultDataArray;
}

JNIEXPORT void JNICALL Java_us_ihmc_ekf_filter_NativeFilterMatrixOpsWrapper_updateState
  (JNIEnv *env, jobject thisObj, jdoubleArray result, jdoubleArray xData, jdoubleArray kData, jdoubleArray rData, jint n, jint m)
{
	jdouble *xDataArray = env->GetDoubleArrayElements(xData, NULL);
	jdouble *kDataArray = env->GetDoubleArrayElements(kData, NULL);
	jdouble *rDataArray = env->GetDoubleArrayElements(rData, NULL);
	MatrixXd x = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(xDataArray, n, 1);
	MatrixXd K = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(kDataArray, n, m);
	MatrixXd r = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(rDataArray, m, 1);

	MatrixXd state = x + K * r;

	jdouble *resultDataArray = new jdouble[n];
	Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(resultDataArray, n, 1) = state;
	env->SetDoubleArrayRegion(result, 0, n, resultDataArray);

	env->ReleaseDoubleArrayElements(xData, xDataArray, 0);
	env->ReleaseDoubleArrayElements(kData, kDataArray, 0);
	env->ReleaseDoubleArrayElements(rData, rDataArray, 0);
	delete resultDataArray;
}
