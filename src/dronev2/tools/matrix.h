#ifndef matrix_h
#define matrix_h

/** © 2024 Keshav Haripersad
 *  Advanced matrix API logic - matrix.h
 *  | This matrix API has many functions, and is designed to be a standalone matrix library.
 * 
 *  Licensed under the MIT license (check LICENSE).
 *  Github link: https://github.com/Keshav11-coder/matrix.h
 */

#include <vector>
#include <functional>

#include "Arduino.h"

// Prints matrix in a formatted manner using Serial.print
inline void printMatrix(const std::vector<std::vector<float>> &matrix)
{
    Serial.println("{");
    for (size_t i = 0; i < matrix.size(); i++)
    {
        Serial.print("  {");
        for (size_t j = 0; j < matrix[i].size(); j++)
        {
            Serial.print(matrix[i][j], 6); // Print with 6 decimal places
            if (j < matrix[i].size() - 1)
            {
                Serial.print(", ");
            }
        }
        Serial.print("}");
        if (i < matrix.size() - 1)
        {
            Serial.println(",");
        }
    }
    Serial.println("\n}");
}

// Requirmements for std::cout matrix prints
// #include <iostream>
// #include <iomanip>

// Prints matrix in a formatted manner using std::cout
// inline void printMatrix(const std::vector<std::vector<float>> &matrix)
// {
//     std::cout << "{" << std::endl;
//     for (size_t i = 0; i < matrix.size(); i++)
//     {
//         std::cout << "  {";
//         for (size_t j = 0; j < matrix[i].size(); j++)
//         {
//             std::cout << std::fixed << std::setprecision(6) << matrix[i][j]; // Print with 6 decimal places
//             if (j < matrix[i].size() - 1)
//             {
//                 std::cout << ", ";
//             }
//         }
//         std::cout << "}";
//         if (i < matrix.size() - 1)
//         {
//             std::cout << "," << std::endl;
//         }
//     }
//     std::cout << "\n}" << std::endl;
// }

class matrix
{
public:
    matrix(int rows, int columns, float value)
    {
        _mat = std::vector<std::vector<float>>(rows, std::vector<float>(columns, value));
        _rows = rows;
        _columns = columns;
    }

    matrix(std::vector<std::vector<float>> mat_)
    {
        _mat = mat_;
        _rows = mat_.size();
        _columns = mat_[0].size();
    }

    matrix(std::initializer_list<std::initializer_list<float>> mat_)
    {
        _mat = std::vector<std::vector<float>>(mat_.begin(), mat_.end());
        _rows = _mat.size();
        _columns = _rows > 0 ? _mat[0].size() : 0;
    }

    std::vector<std::vector<float>> mat() const
    { // Marked as const
        return _mat;
    }

    int rows() { return _rows; }
    int columns() { return _columns; }

    void print() const
    { // Marked as const
        printMatrix(_mat);
    }

    float operator()(int row, int column)
    {
        return _mat[row - 1][column - 1];
    }

    matrix operator*(matrix mat2)
    {
        std::vector<std::vector<float>> a = _mat;
        std::vector<std::vector<float>> b = mat2.mat();
        if (a[0].size() != 0 && b.size() != 0)
        {
            if (a[0].size() == b.size())
            {
                const int aRows = a.size();
                const int aColumns = a[0].size();
                const int bRows = b.size();
                const int bColumns = b[0].size();

                const int rRows = aRows;
                const int rColumns = bColumns;

                std::vector<std::vector<float>> result(aRows, std::vector<float>(bColumns, 0.0f));

                for (int i = 0; i < aRows; i++)
                {
                    for (int j = 0; j < bColumns; j++)
                    {
                        for (int k = 0; k < aColumns; k++)
                        {
                            result[i][j] += a[i][k] * b[k][j];
                        }
                    }
                }

                return matrix(result);
            }
            return _mat;
        }
        return _mat;
    }

    bool operator==(matrix mat2)
    {
        if (_mat == mat2.mat())
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    matrix operator*(float integral)
    {
        std::vector<std::vector<float>> a = _mat;
        std::vector<std::vector<float>> result = _mat;

        for (int i = 0; i < a.size(); i++)
        {
            for (int j = 0; j < a[0].size(); j++)
            {
                result[i][j] = integral * a[i][j];
            }
        }

        return matrix(result);
    }

    matrix operator-(matrix mat2)
    {
        std::vector<std::vector<float>> result = _mat;

        if (result.size() == mat2.mat().size() && result[0].size() == mat2.mat()[0].size())
        {
            for (int i = 0; i < mat2.mat().size(); i++)
            {
                for (int j = 0; j < mat2.mat()[0].size(); j++)
                {
                    result[i][j] = result[i][j] - mat2.mat()[i][j];
                }
            }
            return matrix(result);
        }

        return _mat;
    }

    matrix operator+(matrix mat2)
    {
        std::vector<std::vector<float>> result = _mat;

        if (result.size() == mat2.mat().size() && result[0].size() == mat2.mat()[0].size())
        {
            for (int i = 0; i < mat2.mat().size(); i++)
            {
                for (int j = 0; j < mat2.mat()[0].size(); j++)
                {
                    result[i][j] = result[i][j] + mat2.mat()[i][j];
                }
            }
            return matrix(result);
        }

        return _mat;
    }

    matrix hadamard(matrix mat2)
    {
        std::vector<std::vector<float>> a = _mat;
        std::vector<std::vector<float>> b = mat2.mat();
        if (a.size() != b.size() || a[0].size() != b[0].size())
        {
            return _mat;
        }

        std::vector<std::vector<float>> result(a.size(), std::vector<float>(a[0].size()));

        for (size_t i = 0; i < a.size(); ++i)
        {
            for (size_t j = 0; j < a[0].size(); ++j)
            {
                result[i][j] = a[i][j] * b[i][j];
            }
        }

        return matrix(result);
    }

    matrix T()
    {
        std::vector<std::vector<float>> T;

        for (int i = 0; i < _mat[0].size(); i++)
        {
            T.push_back({});
        }

        for (int j = 0; j < _mat.size(); j++)
        {
            for (int k = 0; k < _mat[0].size(); k++)
            {
                T[k].push_back(_mat[j][k]);
            }
        }

        return matrix(T);
    }

    std::vector<float> flattened()
    {
        std::vector<float> flat;
        for (int i = 0; i < _mat.size(); i++)
        {
            for (int j = 0; j < _mat[0].size(); j++)
            {
                flat.push_back(0.00f);
            }
        }

        for (int i = 0; i < _mat.size(); i++)
        {
            for (int j = 0; j < _mat[0].size(); j++)
            {
                flat[(i * _mat[0].size()) + j] = _mat[i][j];
            }
        }

        return flat;
    }

    matrix operator!() const
    {
        if (_mat.size() != _mat[0].size())
        {
            return matrix(_mat);
        }

        // Initialize an identity matrix of the same size
        std::vector<std::vector<float>> identity(_mat.size(), std::vector<float>(_mat[0].size(), 0));
        for (int i = 0; i < _mat.size(); ++i)
        {
            identity[i][i] = 1.0;
        }

        // Copy of the original matrix
        std::vector<std::vector<float>> mat_copy = _mat;

        // Gaussian elimination
        for (int i = 0; i < _mat.size(); ++i)
        {
            float pivot = mat_copy[i][i];
            if (pivot == 0)
            {
                return matrix(_mat);
            }

            for (int j = 0; j < _mat[0].size(); ++j)
            {
                mat_copy[i][j] /= pivot;
                identity[i][j] /= pivot;
            }

            for (int k = 0; k < _mat.size(); ++k)
            {
                if (k != i)
                {
                    float factor = mat_copy[k][i];
                    for (int j = 0; j < _mat[0].size(); ++j)
                    {
                        mat_copy[k][j] -= factor * mat_copy[i][j];
                        identity[k][j] -= factor * identity[i][j];
                    }
                }
            }
        }

        return matrix(identity);
    }

    matrix pseudoinverse()
    {
        matrix mat = matrix(_mat);
        if (mat.rows() < mat.columns())
        {
            return mat.T() * !(mat * mat.T());
        }
        else if (mat.rows() > mat.columns())
        {
            return !(mat.T() * mat) * mat.T();
        }
        else
        {
            return mat;
        }
    }

    int set(int row, int column, float value_)
    {
        if (row >= 1 && column >= 1)
        {
            if (row <= _mat.size())
            {
                if (column <= _mat[0].size())
                {
                    _mat[row - 1][column - 1] = value_;
                    return 0;
                }
                else
                {
                    return 1;
                }
            }
            else
            {
                return 1;
            }
        }
        else
        {
            return 1;
        }
    }

    std::vector<float> row(int r)
    {
        if (r > 0 && r <= _rows)
        {
        }
        else
        {
            r = 0;
        }

        return _mat[r];
    }

    std::vector<float> column(int c)
    {
        if (c > 0 && c <= _columns)
        {
        }
        else
        {
            c = 0;
        }

        std::vector<float> column = {};

        for (int i = 0; i < _rows - 1; i++)
        {
            column.push_back(_mat[i][c]);
        }

        return column;
    }

    matrix operator()(int r1, int c1, int r2, int c2)
    {
        if (r1 > 0 && r1 <= _rows && c1 > 0 && c1 <= _columns) // point 1 is within bounds
        {
            if (r2 > 0 && r2 <= _rows && c2 > 0 && c2 <= _columns) // point 2 is within bounds
            {
                std::vector<std::vector<float>> region(r2 - r1 + 1, std::vector<float>(c2 - c1 + 1, 1));
                for (int i = r1 - 1, i_r = 0; i <= r2 - 1, i_r <= r2 - r1; i++, i_r++)
                {
                    for (int j = c1 - 1, j_r = 0; j <= c2 - 1, j_r <= c2 - c1; j++, j_r++)
                    {
                        region[i_r][j_r] = _mat[i][j];
                    }
                }
                return matrix(region);
            }
        }
        else
        {
            return matrix(r2 - r1 + 1, c2 - c1 + 1, 0); // return empty (0) matrix, so equations end up being 0, signaling a fault.
        }
    }

    // Previous one (manually written)
    int set(int r1, int c1, int r2, int c2, matrix region)
    {
        if (r1 > 0 && r1 <= _rows && c1 > 0 && c1 <= _columns) // point 1 is within bounds
        {
            if (r2 > 0 && r2 <= _rows && c2 > 0 && c2 <= _columns) // point 2 is within bounds
            {
                for (int i = r1 - 1, i_r = 1; i <= r2 - 1, i_r <= region.rows(); i++, i_r++)
                {
                    for (int j = c1 - 1, j_r = 1; j <= c2 - 1, j_r <= region.columns(); j++, j_r++)
                    {
                        _mat[i][j] = region(i_r, j_r);
                    }
                }
                return 0;
            }
            return 1;
        }
        else
        {
            return 1; // error, out of bounds.
        }
    }

    matrix &forEach(std::function<float(float value, int row, int column)> action)
    {
        for (int i = 0; i < rows(); i++)
        {
            for (int j = 0; j < columns(); j++)
            {
                _mat[i][j] = action(_mat[i][j], i + 1, j + 1);
            }
        }

        return *this;
    }

    matrix &forEach(std::function<float(float value)> action)
    {
        for (int i = 0; i < rows(); i++)
        {
            for (int j = 0; j < columns(); j++)
            {
                _mat[i][j] = action(_mat[i][j]);
            }
        }

        return *this;
    }

    float sum()
    {
        matrix mat(_mat);
        float sum_ = 0;
        mat.forEach([&](float x) mutable -> float
                    {
            sum_ += x;
            return x; });
        return sum_;
    }

    matrix enlarge(int t, int l, int b, int r, float padding)
    {
        if (t >= 0 && l >= 0 && b >= 0 && r >= 0)
        {
            matrix yM = matrix(rows() + t + b, columns() + l + r, padding);
            yM.set(l + 1, t + 1, rows() + t, columns() + l, matrix(_mat));
            return yM;
        }
        return matrix(_mat);
    }

private:
    std::vector<std::vector<float>> _mat;
    int _rows, _columns;
};

#endif