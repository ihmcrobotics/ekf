/*
 * NativeFilterMatrixOps.cpp
 *
 *  Created on: Sep 5, 2018
 *      Author: Georg Wiedebach
 */

#include <jni.h>
#include <stdio.h>
#include <iostream>
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

	MatrixXd ABAt = A * B.selfadjointView<Eigen::Upper>() * A.transpose();

	jdouble *resultDataArray = new jdouble[n * n];
	Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(resultDataArray, n, n) = ABAt;
	env->SetDoubleArrayRegion(result, 0, n * n, resultDataArray);
	env->ReleaseDoubleArrayElements(aData, aDataArray, 0);
	env->ReleaseDoubleArrayElements(bData, bDataArray, 0);
	delete resultDataArray;
}
