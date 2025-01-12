#define _USE_MATH_DEFINES
#include <cmath>
#include <Novice.h>
#include <algorithm>

#include "math/Vector2.h"
#include "math/Vector3.h"
#include "math/Vector4.h"
#include "math/MathUtility.h"

const char kWindowTitle[] = "LE2B_18_ナカガワ_リクト";

void PrintMatrix(const KamataEngine::Vector2& position, const KamataEngine::Matrix4x4& matrix, const char* name) {
	// 行列の名前を描画
	Novice::ScreenPrintf(static_cast<int>(position.x), static_cast<int>(position.y), name);

	// 行列の各行を描画
	for (int i = 0; i < 4; ++i) {
		Novice::ScreenPrintf(static_cast<int>(position.x), static_cast<int>(position.y + 20 * (i + 1)),
			" %5.3f, %5.3f, %5.3f, %5.3f",
			matrix.m[0][i], matrix.m[1][i], matrix.m[2][i], matrix.m[3][i]);
	}
}

namespace KamataEngine {

	///=====================================================///
	///内積の計算
	///=====================================================///
	float Dot(const Vector2& v) { return v.x * v.x + v.y * v.y; }
	float Dot(const Vector2& v1, const Vector2& v2) { return v1.x * v2.x + v1.y * v2.y; }
	float Dot(const Vector3& v) { return v.x * v.x + v.y * v.y + v.z * v.z; }
	float Dot(const Vector3& v1, const Vector3& v2) { return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z; }

	///=====================================================///
	///長さの計算
	///=====================================================///
	float Length(const Vector2& v) { return std::sqrtf(Dot(v)); }
	float Length(const Vector2& v1, const Vector2& v2) { return std::sqrtf(Dot(v1, v2)); }
	float Length(const Vector3& v) { return std::sqrtf(Dot(v)); }
	float Length(const Vector3& v1, const Vector3& v2) { return std::sqrtf(Dot(v1, v2)); }

	///=====================================================///
	///正規化
	///=====================================================///
	Vector2 Normalize(const Vector2& v) {
		float mag = Length(v);
		if (mag != 0.0f) {
			return { v.x / mag, v.y / mag };
		}
		// ゼロベクトルの場合はそのまま返す
		return v;
	}
	Vector3 Normalize(const Vector3& v) {
		float mag = Length(v);
		if (mag != 0.0f) {
			return { v.x / mag, v.y / mag, v.z / mag };
		}
		// ゼロベクトルの場合はそのまま返す
		return v;
	}

	///=====================================================///
	///行列の加法
	///=====================================================///
	Matrix4x4 Add(const Matrix4x4& m1, const Matrix4x4& m2) {

		Matrix4x4 result;

		result.m[0][0] = m1.m[0][0] + m2.m[0][0];
		result.m[0][1] = m1.m[0][1] + m2.m[0][1];
		result.m[0][2] = m1.m[0][2] + m2.m[0][2];
		result.m[0][3] = m1.m[0][3] + m2.m[0][3];
		result.m[1][0] = m1.m[1][0] + m2.m[1][0];
		result.m[1][1] = m1.m[1][1] + m2.m[1][1];
		result.m[1][2] = m1.m[1][2] + m2.m[1][2];
		result.m[1][3] = m1.m[1][3] + m2.m[1][3];
		result.m[2][0] = m1.m[2][0] + m2.m[2][0];
		result.m[2][1] = m1.m[2][1] + m2.m[2][1];
		result.m[2][2] = m1.m[2][2] + m2.m[2][2];
		result.m[2][3] = m1.m[2][3] + m2.m[2][3];
		result.m[3][0] = m1.m[3][0] + m2.m[3][0];
		result.m[3][1] = m1.m[3][1] + m2.m[3][1];
		result.m[3][2] = m1.m[3][2] + m2.m[3][2];
		result.m[3][3] = m1.m[3][3] + m2.m[3][3];

		return result;
	}

	///=====================================================///
	///行列の減法
	///=====================================================///
	Matrix4x4 Subject(const Matrix4x4& m1, const Matrix4x4& m2) {

		Matrix4x4 result;

		result.m[0][0] = m1.m[0][0] - m2.m[0][0];
		result.m[0][1] = m1.m[0][1] - m2.m[0][1];
		result.m[0][2] = m1.m[0][2] - m2.m[0][2];
		result.m[0][3] = m1.m[0][3] - m2.m[0][3];
		result.m[1][0] = m1.m[1][0] - m2.m[1][0];
		result.m[1][1] = m1.m[1][1] - m2.m[1][1];
		result.m[1][2] = m1.m[1][2] - m2.m[1][2];
		result.m[1][3] = m1.m[1][3] - m2.m[1][3];
		result.m[2][0] = m1.m[2][0] - m2.m[2][0];
		result.m[2][1] = m1.m[2][1] - m2.m[2][1];
		result.m[2][2] = m1.m[2][2] - m2.m[2][2];
		result.m[2][3] = m1.m[2][3] - m2.m[2][3];
		result.m[3][0] = m1.m[3][0] - m2.m[3][0];
		result.m[3][1] = m1.m[3][1] - m2.m[3][1];
		result.m[3][2] = m1.m[3][2] - m2.m[3][2];
		result.m[3][3] = m1.m[3][3] - m2.m[3][3];

		return result;
	}

	///=====================================================///
	///行列の積
	///=====================================================///
	Matrix4x4 Multiply(const Matrix4x4& m1, const Matrix4x4& m2) {

		Matrix4x4 answer = {};
		for (int x = 0; x < 4; ++x) {
			for (int y = 0; y < 4; ++y) {

				answer.m[x][y] = 0;
				for (int z = 0; z < 4; ++z) {

					answer.m[x][y] += m1.m[x][z] * m2.m[z][y];
				}
			}
		}

		return answer;
	}

	///-------------------------------------------/// 
	/// 任意軸回転行列の関数
	///-------------------------------------------///
	Matrix4x4 MakeRotateAxisAngle(const Vector3& axis, float angle) {
		// 回転軸を正規化
		Vector3 normalizedAxis = Normalize(axis);

		float x = normalizedAxis.x;
		float y = normalizedAxis.y;
		float z = normalizedAxis.z;

		float cosTheta = std::cos(angle);
		float sinTheta = std::sin(angle);
		float oneMinusCosTheta = 1.0f - cosTheta;

		Matrix4x4 rotationMatrix;

		// ロドリゲスの回転公式に基づく計算
		rotationMatrix.m[0][0] = cosTheta + x * x * oneMinusCosTheta;
		rotationMatrix.m[0][1] = x * y * oneMinusCosTheta - z * sinTheta;
		rotationMatrix.m[0][2] = x * z * oneMinusCosTheta + y * sinTheta;
		rotationMatrix.m[0][3] = 0.0f;

		rotationMatrix.m[1][0] = y * x * oneMinusCosTheta + z * sinTheta;
		rotationMatrix.m[1][1] = cosTheta + y * y * oneMinusCosTheta;
		rotationMatrix.m[1][2] = y * z * oneMinusCosTheta - x * sinTheta;
		rotationMatrix.m[1][3] = 0.0f;

		rotationMatrix.m[2][0] = z * x * oneMinusCosTheta - y * sinTheta;
		rotationMatrix.m[2][1] = z * y * oneMinusCosTheta + x * sinTheta;
		rotationMatrix.m[2][2] = cosTheta + z * z * oneMinusCosTheta;
		rotationMatrix.m[2][3] = 0.0f;

		rotationMatrix.m[3][0] = 0.0f;
		rotationMatrix.m[3][1] = 0.0f;
		rotationMatrix.m[3][2] = 0.0f;
		rotationMatrix.m[3][3] = 1.0f;

		return rotationMatrix;
	}

	///-------------------------------------------/// 
	/// ある方向からある方向への回転
	///-------------------------------------------///
	Matrix4x4 DirectionToDirection(const Vector3& from, const Vector3& to) {
		// 入力ベクトルが正規化されていない場合は正規化する
		Vector3 fromNormalized = Normalize(from);
		Vector3 toNormalized = Normalize(to);

		// ベクトルがほぼ同じ場合は単位行列を返す
		float dotProduct = fromNormalized.x * toNormalized.x + fromNormalized.y * toNormalized.y + fromNormalized.z * toNormalized.z;
		if (fabs(dotProduct - 1.0f) < 1e-6f) {
			Matrix4x4 identity;
			for (int i = 0; i < 4; ++i) {
				for (int j = 0; j < 4; ++j) {
					identity.m[i][j] = (i == j) ? 1.0f : 0.0f;
				}
			}
			return identity;
		}

		// ベクトルが反対方向の場合は適当な回転軸を使用して180度回転
		if (fabs(dotProduct + 1.0f) < 1e-6f) {
			Vector3 axis = { 1.0f, 0.0f, 0.0f };
			if(fabs(fromNormalized.x) < fabs(fromNormalized.y) && fabs(fromNormalized.x) < fabs(fromNormalized.z)) {
				axis = { 0.0f, -fromNormalized.z, fromNormalized.y }; // X 軸に垂直
			} else if (fabs(fromNormalized.y) < fabs(fromNormalized.z)) {
				axis = { -fromNormalized.z, 0.0f, fromNormalized.x }; // Y 軸に垂直
			} else {
				axis = { -fromNormalized.y, fromNormalized.x, 0.0f }; // Z 軸に垂直
			}
			float length = Length(axis);
			axis = { axis.x / length, axis.y / length, axis.z / length };

			Matrix4x4 result = {};
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                result.m[i][j] = (i == j) ? -1.0f : 0.0f;
            }
        }
		axis = Normalize(axis);
		float c = -1.0f;
		float s = 0.0f;
		float t = 1.0f - c;

		Matrix4x4 rotation = {};
		rotation.m[0][0] = t * axis.x * axis.x + c;
		rotation.m[0][1] = t * axis.x * axis.y - s * axis.z;
		rotation.m[0][2] = t * axis.x * axis.z + s * axis.y;
		rotation.m[1][0] = t * axis.x * axis.y + s * axis.z;
		rotation.m[1][1] = t * axis.y * axis.y + c;
		rotation.m[1][2] = t * axis.y * axis.z - s * axis.x;
		rotation.m[2][0] = t * axis.x * axis.z - s * axis.y;
		rotation.m[2][1] = t * axis.y * axis.z + s * axis.x;
		rotation.m[2][2] = t * axis.z * axis.z + c;
		rotation.m[3][3] = 1.0f;

		return rotation;
		}

		/// ===一般的なケース: 回転軸と角度を計算=== ///
		Vector3 rotationAxis = {
			fromNormalized.y * toNormalized.z - fromNormalized.z * toNormalized.y,
			fromNormalized.z * toNormalized.x - fromNormalized.x * toNormalized.z,
			fromNormalized.x * toNormalized.y - fromNormalized.y * toNormalized.x
		};
		float axisLength = Length(rotationAxis);
		rotationAxis = { rotationAxis.x / axisLength, rotationAxis.y / axisLength, rotationAxis.z / axisLength };

		float angle = acosf(std::fmax(-1.0f, std::fmin(1.0f, dotProduct)));

		// 回転行列を生成 (ロドリゲスの回転公式を使用)
		Matrix4x4 rotation = {};
		float c = cosf(angle);
		float s = sinf(angle);
		float t = 1.0f - c;

		rotation.m[0][0] = t * rotationAxis.x * rotationAxis.x + c;
		rotation.m[0][1] = t * rotationAxis.x * rotationAxis.y - s * rotationAxis.z;
		rotation.m[0][2] = t * rotationAxis.x * rotationAxis.z + s * rotationAxis.y;
		rotation.m[1][0] = t * rotationAxis.x * rotationAxis.y + s * rotationAxis.z;
		rotation.m[1][1] = t * rotationAxis.y * rotationAxis.y + c;
		rotation.m[1][2] = t * rotationAxis.y * rotationAxis.z - s * rotationAxis.x;
		rotation.m[2][0] = t * rotationAxis.x * rotationAxis.z - s * rotationAxis.y;
		rotation.m[2][1] = t * rotationAxis.y * rotationAxis.z + s * rotationAxis.x;
		rotation.m[2][2] = t * rotationAxis.z * rotationAxis.z + c;
		rotation.m[3][3] = 1.0f;

		return rotation;
	}
}

// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, 1280, 720);

	// キー入力結果を受け取る箱
	char keys[256] = {0};
	char preKeys[256] = {0};

	KamataEngine::Vector3 from0 = { 1.0f, 0.7f, 0.5f };
	KamataEngine::Vector3 to0 = { -from0.x, -from0.y, -from0.z };
	KamataEngine::Vector3 from1 = { -0.6f, 0.9f, 0.2f };
	KamataEngine::Vector3 to1 = { 0.4f, 0.7f, -0.5f };
	KamataEngine::Matrix4x4 rotateMatrix0 = KamataEngine::DirectionToDirection(
		{ 1.0f, 0.0f, 0.0f }, { -1.0f, 0.0f, 0.0f });
	KamataEngine::Matrix4x4 rotateMatrix1 = KamataEngine::DirectionToDirection(from0, to0);
	KamataEngine::Matrix4x4 rotateMatrix2 = KamataEngine::DirectionToDirection(from1, to1);



	// ウィンドウの×ボタンが押されるまでループ
	while (Novice::ProcessMessage() == 0) {
		// フレームの開始
		Novice::BeginFrame();

		// キー入力を受け取る
		memcpy(preKeys, keys, 256);
		Novice::GetHitKeyStateAll(keys);

		///
		/// ↓更新処理ここから
		///

		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///

		PrintMatrix({0.0f, 0.0f}, rotateMatrix0, "rotateMatrix0");
		PrintMatrix({ 0.0f, 20.0f * 5.0f }, rotateMatrix1, "rotateMatrix1");
		PrintMatrix({ 0.0f, 20.0f * 10.0f }, rotateMatrix2, "rotateMatrix2");

		///
		/// ↑描画処理ここまで
		///

		// フレームの終了
		Novice::EndFrame();

		// ESCキーが押されたらループを抜ける
		if (preKeys[DIK_ESCAPE] == 0 && keys[DIK_ESCAPE] != 0) {
			break;
		}
	}

	// ライブラリの終了
	Novice::Finalize();
	return 0;
}
