#define _USE_MATH_DEFINES
#include <cmath>
#include <Novice.h>
#include <string>
#include <stdio.h>

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
			matrix.m[i][0], matrix.m[i][1], matrix.m[i][2], matrix.m[i][3]);
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
}

// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, 1280, 720);

	// キー入力結果を受け取る箱
	char keys[256] = {0};
	char preKeys[256] = {0};

	KamataEngine::Vector3 axis = { 1.0f, 1.0f, 1.0f };
	float angle = 0.44f;
	KamataEngine::Matrix4x4 rotateMatrix = MakeRotateAxisAngle(axis, angle);

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

		PrintMatrix({0.0f, 0.0f}, rotateMatrix, "rotateMatrix");

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
