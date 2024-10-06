#ifndef nnl_h
#define nnl_h

#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <tuple>

// I'm so sorry

namespace neural
{
    struct matrix
    {
    private:
    public:
        std::vector<std::vector<float>> multiply(std::vector<std::vector<float>> a, std::vector<std::vector<float>> b)
        {
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

                    return result;
                }
            }
        }

        std::vector<std::vector<float>> multiply(float a, std::vector<std::vector<float>> b)
        {
            std::vector<std::vector<float>> result = b;

            for (int i = 0; i < b.size(); i++)
            {
                for (int j = 0; j < b[0].size(); j++)
                {
                    result[i][j] = a * b[i][j];
                }
            }

            return result;
        }

        std::vector<std::vector<float>> difference(std::vector<std::vector<float>> a, std::vector<std::vector<float>> b)
        {
            std::vector<std::vector<float>> result = a;

            if (a.size() == b.size() && a[0].size() == b[0].size())
            {
                for (int i = 0; i < b.size(); i++)
                {
                    for (int j = 0; j < b[0].size(); j++)
                    {
                        result[i][j] = result[i][j] - b[i][j];
                    }
                }
                return result;
            }

            return a;
        }

        std::vector<std::vector<float>> sum(std::vector<std::vector<float>> a, std::vector<std::vector<float>> b)
        {
            std::vector<std::vector<float>> result = a;

            if (a.size() == b.size() && a[0].size() == b[0].size())
            {
                for (int i = 0; i < b.size(); i++)
                {
                    for (int j = 0; j < b[0].size(); j++)
                    {
                        result[i][j] = result[i][j] + b[i][j];
                    }
                }
                return result;
            }

            return a;
        }

        std::vector<std::vector<float>> multiply_hadamard(const std::vector<std::vector<float>> &a, const std::vector<std::vector<float>> &b)
        {
            if (a.size() != b.size() || a[0].size() != b[0].size())
            {
                return {{0}};
            }

            std::vector<std::vector<float>> result(a.size(), std::vector<float>(a[0].size()));

            for (size_t i = 0; i < a.size(); ++i)
            {
                for (size_t j = 0; j < a[0].size(); ++j)
                {
                    result[i][j] = a[i][j] * b[i][j];
                }
            }

            return result;
        }

        std::vector<std::vector<float>> transpose(std::vector<std::vector<float>> matrix)
        {
            std::vector<std::vector<float>> matrixT;

            for (int i = 0; i < matrix[0].size(); i++) // create n(Columns) amount of rows
            {
                matrixT.push_back({});
            }

            for (int j = 0; j < matrix.size(); j++) // go through the rows
            {
                for (int k = 0; k < matrix[0].size(); k++) // go through the columns
                {
                    matrixT[k].push_back(matrix[j][k]); // the newly built matrix[ the column (bc r = c) index] . push_back(matrix[ the row index ] [ column index ])
                }
            }

            return matrixT;
        }

        std::vector<std::vector<float>> from(std::vector<float> flatmatrix, int rows, int columns)
        {
            std::vector<std::vector<float>> matrix;
            if (rows * columns == flatmatrix.size())
            {
                for (int i = 0; i < rows; i++)
                {
                    matrix.push_back({});
                    for (int j = 0; j < columns; j++)
                    {
                        matrix[i].push_back(0.00f);
                    }
                }

                for (int i = 0; i < rows; i++)
                {
                    for (int j = 0; j < columns; j++)
                    {
                        matrix[i][j] = flatmatrix[(i * columns) + (j)];
                    }
                }

                return matrix;
            }
            else
            {
                return {{0}};
            }
        }

        std::vector<float> flatten(std::vector<std::vector<float>> matrix)
        {
            std::vector<float> matrixFlattened;
            for (int i = 0; i < matrix.size(); i++)
            {
                for (int j = 0; j < matrix[0].size(); j++)
                {
                    matrixFlattened.push_back(0.00f);
                }
            }

            for (int i = 0; i < matrix.size(); i++)
            {
                for (int j = 0; j < matrix[0].size(); j++)
                {
                    matrixFlattened[(i * matrix[0].size()) + j] = matrix[i][j];
                }
            }

            return matrixFlattened;
        }

        std::vector<std::vector<float>> zeros(int rows, int columns)
        {
            std::vector<std::vector<float>> mat_zeros(rows, std::vector<float>(columns));
            return mat_zeros;
        }

        std::vector<std::vector<float>> zeros(std::vector<std::vector<float>> reference)
        {
            std::vector<std::vector<float>> mat_zeros(reference.size(), std::vector<float>(reference[0].size()));
            return mat_zeros;
        }

        int size(float **matrix)
        {
            return (sizeof(matrix) / sizeof(matrix[0]));
        }
    } matrix;

    struct sample
    {
    private:
        std::vector<std::vector<float>> _input;
        std::vector<std::vector<std::vector<std::vector<float>>>> _layers;

    public:
        sample(std::vector<std::vector<float>> input_, std::vector<std::vector<std::vector<std::vector<float>>>> layers_)
        {
            _input = input_;
            _layers = layers_;
        }

        std::vector<std::vector<float>> input()
        {
            return _input;
        }

        void set_input(std::vector<std::vector<float>> __input)
        {
            _input = __input;
        }

        std::vector<std::vector<std::vector<std::vector<float>>>> layers()
        {
            return _layers;
        }
    };

    std::vector<std::vector<float>> forward_pass(std::vector<std::vector<float>> x, std::vector<std::vector<float>> y, std::vector<std::vector<float>> b)
    {
        std::vector<std::vector<float>> neurons = matrix.sum(matrix.multiply(y, x), b);
        return neurons;
    }

    float lerp(float A, float B, float t)
    {
        return A + (B - A) * t;
    }

    std::vector<float> sigmoid(std::vector<float> input)
    {
        std::vector<float> activated;
        for (int i = 0; i < input.size(); i++)
        {
            activated.push_back(1 / (1 + exp(-input[i])));
        }
        return activated;
    }

    std::vector<std::vector<float>> sigmoid(std::vector<std::vector<float>> input)
    {
        std::vector<std::vector<float>> activated = input;
        for (int i = 0; i < input.size(); i++)
        {
            for (int j = 0; j < input[0].size(); j++)
            {
                activated[i][j] = 1 / (1 + exp(-input[i][j]));
            }
        }
        return activated;
    }

    std::vector<float> sigmoid_derivative(std::vector<float> input)
    {
        std::vector<float> activated;
        for (int i = 0; i < input.size(); i++)
        {
            activated.push_back(input[i] * (1 - input[i]));
        }
        return activated;
    }

    std::vector<std::vector<float>> sigmoid_derivative(std::vector<std::vector<float>> input)
    {
        std::vector<std::vector<float>> activated = input;
        for (int i = 0; i < input.size(); i++)
        {
            for (int j = 0; j < input[0].size(); j++)
            {
                activated[i][j] = input[i][j] * (1 - input[i][j]);
            }
        }
        return activated;
    }

    std::vector<float> softmax(std::vector<float> logits)
    {
        std::vector<float> exponentials = logits;
        std::vector<float> probabilities = logits;
        float logitsSum = 0;
        for (int i = 0; i < logits.size(); i++)
        {
            exponentials[i] = exp(logits[i]);
            logitsSum += exponentials[i];
        }
        for (int i = 0; i < logits.size(); i++)
        {
            probabilities[i] = exponentials[i] / logitsSum;
        }
        return probabilities;
    }

    std::vector<std::vector<float>> softmax_derivative(std::vector<float> logits)
    {
        std::vector<std::vector<float>> jb_mat = neural::matrix.zeros(logits.size(), logits.size());
        for (int i = 0; i < jb_mat.size(); i++)
        {
            jb_mat[i][i] = logits[i] * (1 - logits[i]);
            for (int j = 0; j < jb_mat[0].size(); j++)
            {
                if (i != j)
                {
                    jb_mat[i][j] = -logits[i] * logits[j];
                }
            }
        }
        return jb_mat;
    }

    std::vector<std::vector<float>> softmax(std::vector<std::vector<float>> logits)
    {
        std::vector<float> logitsFlat = matrix.flatten(logits);
        std::vector<float> exponentials = logitsFlat;
        std::vector<float> probabilities = logitsFlat;
        float logitsSum = 0;
        for (int i = 0; i < logitsFlat.size(); i++)
        {
            exponentials[i] = exp(logitsFlat[i]);
            logitsSum += exponentials[i];
        }
        for (int i = 0; i < logitsFlat.size(); i++)
        {
            probabilities[i] = exponentials[i] / logitsSum;
        }
        return matrix.from(probabilities, logitsFlat.size(), 1);
    }

    std::vector<std::vector<float>> softmax_derivative(std::vector<std::vector<float>> logits)
    {
        std::vector<float> logitsFlat = matrix.flatten(logits);
        std::vector<std::vector<float>> jb_mat = neural::matrix.zeros(logitsFlat.size(), logitsFlat.size());
        for (int i = 0; i < jb_mat.size(); i++)
        {
            jb_mat[i][i] = logitsFlat[i] * (1 - logitsFlat[i]);
            for (int j = 0; j < jb_mat[0].size(); j++)
            {
                if (i != j)
                {
                    jb_mat[i][j] = -logitsFlat[i] * logitsFlat[j];
                }
            }
        }
        return jb_mat;
    }

    std::vector<float> relu(std::vector<float> input)
    {
        std::vector<float> activated;
        for (int i = 0; i < input.size(); i++)
        {
            if (input[i] > 0)
            {
                activated.push_back(input[i]);
            }
            else if (input[i] <= 0)
            {
                activated.push_back(0);
            }
        }
        return activated;
    }

    std::vector<float> relu_derivative(std::vector<float> input)
    {
        std::vector<float> activated;
        for (int i = 0; i < input.size(); i++)
        {
            if (input[i] > 0)
            {
                activated.push_back(1);
            }
            else if (input[i] <= 0)
            {
                activated.push_back(0);
            }
        }
        return activated;
    }

    std::vector<std::vector<float>> relu(std::vector<std::vector<float>> input)
    {
        std::vector<std::vector<float>> activated = input;
        for (int i = 0; i < input.size(); i++)
        {
            for (int j = 0; j < input[0].size(); j++)
            {
                if (input[i][j] > 0)
                {
                    activated[i][j] = input[i][j];
                }
                else if (input[i][j] <= 0)
                {
                    activated[i][j] = 0.00f;
                }
            }
        }
        return activated;
    }

    std::vector<std::vector<float>> relu_derivative(std::vector<std::vector<float>> input)
    {
        std::vector<std::vector<float>> activated = input;
        for (int i = 0; i < input.size(); i++)
        {
            for (int j = 0; j < input[0].size(); j++)
            {
                if (input[i][j] > 0)
                {
                    activated[i][j] = 1.00f;
                }
                else if (input[i][j] <= 0)
                {
                    activated[i][j] = 0.00f;
                }
            }
        }
        return activated;
    }

    float loss(std::vector<float> output, std::vector<float> target)
    {
        float loss_;
        for (int i = 0; i < output.size(); i++)
        {
            loss_ += (output[i] - target[i]) * (output[i] - target[i]);
        }
        return .5 * loss_;
    }

    std::vector<float> gradientloss(std::vector<float> output, std::vector<float> target)
    {
        std::vector<float> gradient;
        return gradient;
    }

    std::vector<float> createWeights(int amount)
    {
        std::vector<float> weights;
        for (int i = 0; i < amount; i++)
        {
            weights.push_back(static_cast<float>(rand()) / static_cast<float>(RAND_MAX));
        }
        return weights;
    }

    std::vector<std::vector<float>> createWeights_mat(int rows, int columns)
    {
        std::vector<std::vector<float>> weights(rows, std::vector<float>(columns, 0.0));
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < columns; j++)
            {
                weights[i][j] = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
            }
        }
        return weights;
    }

    std::vector<std::vector<float>> feedforward(std::vector<std::vector<float>> input, std::vector<std::vector<std::vector<std::vector<float>>>> layers)
    {
        std::vector<std::vector<std::vector<float>>> a;
        std::vector<std::vector<std::vector<float>>> z;
        for (int i = 0; i < layers.size(); i++)
        {
            if (i == 0)
            {
                a.push_back(sigmoid(forward_pass(input, layers[i][0], layers[i][1])));
                z.push_back(forward_pass(input, layers[i][0], layers[i][1]));
            }
            else if (i != 0)
            {
                a.push_back(sigmoid(forward_pass(a[i - 1], layers[i][0], layers[i][1])));
                z.push_back(forward_pass(a[i - 1], layers[i][0], layers[i][1]));
            }
        }
        return a[a.size() - 1];
    }

    std::vector<std::vector<float>> feedforward(std::vector<std::vector<float>> input, std::vector<int> layersNeurons)
    {
        std::vector<std::vector<std::vector<float>>> a;
        for (int i = 0; i < layersNeurons.size(); i++)
        {
            if (i == 0)
            {
                a.push_back(sigmoid(forward_pass(input, createWeights_mat(layersNeurons[i], input.size()), createWeights_mat(layersNeurons[i], 1))));
            }
            else if (i != 0)
            {
                a.push_back(sigmoid(forward_pass(a[i - 1], createWeights_mat(layersNeurons[i], a[i - 1].size()), createWeights_mat(layersNeurons[i], 1))));
            }
        }
        return a[a.size() - 1];
    }

    std::vector<std::vector<std::vector<float>>> feedforward_a(std::vector<std::vector<float>> input, std::vector<std::vector<std::vector<std::vector<float>>>> layers)
    {
        std::vector<std::vector<std::vector<float>>> a;
        std::vector<std::vector<std::vector<float>>> z;
        for (int i = 0; i < layers.size(); i++)
        {
            if (i == 0)
            {
                a.push_back(sigmoid(forward_pass(input, layers[i][0], layers[i][1])));
                z.push_back(forward_pass(input, layers[i][0], layers[i][1]));
            }
            else if (i != 0)
            {
                a.push_back(sigmoid(forward_pass(a[i - 1], layers[i][0], layers[i][1])));
                z.push_back(forward_pass(a[i - 1], layers[i][0], layers[i][1]));
            }
        }
        a.insert(a.begin(), {input});
        return a;
    }

    std::vector<std::vector<std::vector<float>>> feedforward_z(std::vector<std::vector<float>> input, std::vector<std::vector<std::vector<std::vector<float>>>> layers)
    {
        std::vector<std::vector<std::vector<float>>> a;
        std::vector<std::vector<std::vector<float>>> z;
        for (int i = 0; i < layers.size(); i++)
        {
            if (i == 0)
            {
                a.push_back(sigmoid(forward_pass(input, layers[i][0], layers[i][1])));
                z.push_back(forward_pass(input, layers[i][0], layers[i][1]));
            }
            else if (i != 0)
            {
                a.push_back(sigmoid(forward_pass(a[i - 1], layers[i][0], layers[i][1])));
                z.push_back(forward_pass(a[i - 1], layers[i][0], layers[i][1]));
            }
        }
        z.insert(z.begin(), {input});
        return z;
    }

    std::vector<std::vector<std::vector<std::vector<float>>>> backward_pass(std::vector<std::vector<float>> input, std::vector<std::vector<std::vector<std::vector<float>>>> layers, std::vector<std::vector<float>> output, std::vector<std::vector<float>> target)
    {
        std::vector<std::vector<std::vector<float>>> layerErrorTerms(layers.size(), std::vector<std::vector<float>>(1, std::vector<float>(1, 0)));
        std::vector<std::vector<std::vector<float>>> layerGradientWeights(layers.size(), std::vector<std::vector<float>>(1, std::vector<float>(1, 0)));
        std::vector<std::vector<std::vector<float>>> layerGradientBiases(layers.size(), std::vector<std::vector<float>>(1, std::vector<float>(1, 0)));

        std::vector<std::vector<std::vector<float>>> calculated_a = feedforward_a(input, layers);

        for (int i = layers.size() - 1; i > -1; i--)
        {
            if (i == layers.size() - 1)
            {
                layerErrorTerms[i] = matrix.multiply_hadamard(matrix.difference(output, target), sigmoid_derivative(output));
                layerGradientWeights[i] = matrix.multiply(layerErrorTerms[i], matrix.transpose(calculated_a[i + 1])); // matrix.multiply(layerErrorTerms[i], calculated_a[i]); // current error term times the previous output
                layerGradientBiases[i] = layerErrorTerms[i];
            }
            else if (i != layers.size() - 1)
            {
                layerErrorTerms[i] = matrix.multiply_hadamard(matrix.multiply(matrix.transpose(layers[i + 1][0]), layerErrorTerms[i + 1]), sigmoid_derivative(calculated_a[i + 1]));
                layerGradientWeights[i] = matrix.multiply(layerErrorTerms[i], matrix.transpose(calculated_a[i])); // current error term times the previous output
                layerGradientBiases[i] = layerErrorTerms[i];
            }
        }

        return {layerErrorTerms, layerGradientWeights, layerGradientBiases};
    }

    sample backpropagate(sample _sample_, std::vector<std::vector<float>> output, std::vector<std::vector<float>> target, float _n_)
    {
        std::vector<std::vector<float>> input = _sample_.input();
        std::vector<std::vector<std::vector<std::vector<float>>>> layers = _sample_.layers();

        std::vector<std::vector<std::vector<float>>> calculated_a = feedforward_a(input, layers);
        std::vector<std::vector<std::vector<std::vector<float>>>> layers_new = layers;

        std::vector<std::vector<std::vector<std::vector<float>>>> backwardPass = backward_pass(input, layers, output, target);
        std::vector<std::vector<std::vector<float>>> layerErrorTerms = backwardPass[0];
        std::vector<std::vector<std::vector<float>>> layerGradientWeights = backwardPass[1];
        std::vector<std::vector<std::vector<float>>> layerGradientBiases = backwardPass[2];

        for (int i = layers.size() - 1; i > -1; i--)
        {
            layers_new[i][0] = matrix.difference(layers[i][0], matrix.multiply(_n_, matrix.multiply(layerErrorTerms[i], matrix.transpose(calculated_a[i]))));
            layers_new[i][1] = matrix.difference(layers[i][1], matrix.multiply(_n_, layerErrorTerms[i]));
        }

        sample __sample__(input, layers_new);
        return __sample__;
    }

    sample backpropagate_batch(sample _sample_, std::vector<std::vector<std::vector<float>>> batch_inputs, std::vector<std::vector<std::vector<float>>> batch_targets, float _n_)
    {
        std::vector<std::vector<float>> input = _sample_.input();
        std::vector<std::vector<std::vector<std::vector<float>>>> layers = _sample_.layers();

        std::vector<std::vector<std::vector<std::vector<float>>>> layers_new = layers;

        std::vector<std::vector<std::vector<float>>> accumulated_layerErrorTerms(layers.size());
        std::vector<std::vector<std::vector<float>>> accumulated_layerGradientWeights(layers.size());
        std::vector<std::vector<std::vector<float>>> accumulated_layerGradientBiases(layers.size());

        int batch_size = batch_inputs.size();

        for (int i = 0; i < layers.size(); i++)
        {
            accumulated_layerErrorTerms[i] = matrix.zeros(layers[i][0]);
            accumulated_layerGradientWeights[i] = matrix.zeros(layers[i][0]);
            accumulated_layerGradientBiases[i] = matrix.zeros(layers[i][1]);
        }

        for (int b = 0; b < batch_size; b++)
        {
            std::vector<std::vector<float>> output = feedforward(batch_inputs[b], layers);
            std::vector<std::vector<float>> target = batch_targets[b];

            std::vector<std::vector<std::vector<float>>> calculated_a = feedforward_a(batch_inputs[b], layers);

            std::vector<std::vector<std::vector<std::vector<float>>>> backwardPass = backward_pass(batch_inputs[b], layers, output, target);
            std::vector<std::vector<std::vector<float>>> layerErrorTerms = backwardPass[0];
            std::vector<std::vector<std::vector<float>>> layerGradientWeights = backwardPass[1];
            std::vector<std::vector<std::vector<float>>> layerGradientBiases = backwardPass[2];

            for (int a = 0; a < layers.size(); a++)
            {
                accumulated_layerErrorTerms[a] = matrix.sum(accumulated_layerErrorTerms[a], layerErrorTerms[a]);
                accumulated_layerGradientWeights[a] = matrix.sum(accumulated_layerGradientWeights[a], layerGradientWeights[a]);
                accumulated_layerGradientBiases[a] = matrix.sum(accumulated_layerGradientBiases[a], layerGradientBiases[a]);
            }
        }

        for (int i = layers.size() - 1; i > -1; i--)
        {
            accumulated_layerGradientWeights[i] = matrix.multiply(1.0f / batch_size, accumulated_layerGradientWeights[i]);
            accumulated_layerGradientBiases[i] = matrix.multiply(1.0f / batch_size, accumulated_layerGradientBiases[i]);

            layers_new[i][0] = matrix.difference(layers[i][0], matrix.multiply(_n_, accumulated_layerGradientWeights[i]));
            layers_new[i][1] = matrix.difference(layers[i][1], matrix.multiply(_n_, accumulated_layerGradientBiases[i]));
        }

        sample __sample__(input, layers_new);
        return __sample__;
    }

    sample mutate(sample _sample_, float amount = 1)
    {
        std::vector<std::vector<float>> input = _sample_.input();
        std::vector<std::vector<std::vector<std::vector<float>>>> layers = _sample_.layers();
        std::vector<std::vector<std::vector<std::vector<float>>>> layers_new = _sample_.layers();

        for (int i = 0; i < layers.size(); i++)
        {
            for (int j = 0; j < layers[i][0].size(); j++)
            {
                for (int k = 0; k < layers[i][0][0].size(); k++)
                {
                    layers_new[i][0][j][k] = lerp(
                        layers[i][0][j][k],
                        createWeights(1)[0],
                        amount);
                }
            }
            for (int j = 0; j < layers[i][1].size(); j++)
            {
                for (int k = 0; k < layers[i][1][0].size(); k++)
                {
                    layers_new[i][1][j][k] = lerp(
                        layers[i][1][j][k],
                        createWeights(1)[0],
                        amount);
                }
            }
        }

        sample __sample__(input, layers_new);
        return __sample__;
    }

    enum activation
    {
        Softmax,
        Sigmoid,
        ReLU
    };

    class layer
    {
    private:
    public:
        activation layer_activation;
        std::vector<std::vector<float>> weights;
        std::vector<std::vector<float>> biases;

        layer(activation activation_, std::vector<std::vector<float>> weights_, std::vector<std::vector<float>> biases_)
        {
            layer_activation = activation_;
            weights = weights_;
            biases = biases_;
        }

        int update_weights(std::vector<std::vector<float>> upd_weights)
        {
            weights = upd_weights;
            return 0;
        }

        int update_biases(std::vector<std::vector<float>> upd_biases)
        {
            biases = upd_biases;
            return 0;
        }
    };

    class network
    {
    private:
    public:
        std::vector<std::vector<float>> input;
        std::vector<layer *> layers;

        network(std::vector<std::vector<float>> input_, std::vector<layer *> layers_)
        {
            input = input_;
            layers = layers_;
        }
    };

    class network_v2
    {
    private:
    public:
        std::vector<std::vector<float>> input;
        std::vector<layer *> layers;

        template <typename... Layers>
        network_v2(std::vector<std::vector<float>> input_, Layers... layers_)
        {
            input = input_;
            (layers.push_back(layers_), ...);
        }
    };

    std::vector<std::vector<float>> activate(std::vector<std::vector<float>> matrix, activation activation_)
    {
        if (activation_ == activation::Sigmoid)
        {
            return sigmoid(matrix);
        }
        else if (activation_ == activation::Softmax)
        {
            return softmax(matrix);
        }
        else if (activation_ == activation::ReLU)
        {
            return relu(matrix);
        }
    }

    std::vector<std::vector<float>> activate_derivative(std::vector<std::vector<float>> matrix, activation activation_)
    {
        if (activation_ == activation::Sigmoid)
        {
            return sigmoid_derivative(matrix);
        }
        else if (activation_ == activation::Softmax)
        {
            return softmax_derivative(matrix);
        }
        else if (activation_ == activation::ReLU)
        {
            return relu_derivative(matrix);
        }
    }


    //Network V1 support'
    std::vector<std::vector<float>> feedforward(network *network_)
    {
        std::vector<std::vector<std::vector<float>>> a;
        std::vector<std::vector<std::vector<float>>> z;
        for (int i = 0; i < network_->layers.size(); i++)
        {
            if (i == 0)
            {
                a.push_back(activate(forward_pass(network_->input, network_->layers[i]->weights, network_->layers[i]->biases), network_->layers[i]->layer_activation));
                z.push_back(forward_pass(network_->input, network_->layers[i]->weights, network_->layers[i]->biases));
            }
            else if (i != 0)
            {
                a.push_back(activate(forward_pass(a[i - 1], network_->layers[i]->weights, network_->layers[i]->biases), network_->layers[i]->layer_activation));
                z.push_back(forward_pass(a[i - 1], network_->layers[i]->weights, network_->layers[i]->biases));
            }
        }
        return a[a.size() - 1];
    }

    std::vector<std::vector<std::vector<float>>> feedforward_a(network *network_)
    {
        std::vector<std::vector<std::vector<float>>> a;
        std::vector<std::vector<std::vector<float>>> z;
        for (int i = 0; i < network_->layers.size(); i++)
        {
            if (i == 0)
            {
                a.push_back(activate(forward_pass(network_->input, network_->layers[i]->weights, network_->layers[i]->biases), network_->layers[i]->layer_activation));
                z.push_back(forward_pass(network_->input, network_->layers[i]->weights, network_->layers[i]->biases));
            }
            else if (i != 0)
            {
                a.push_back(activate(forward_pass(a[i - 1], network_->layers[i]->weights, network_->layers[i]->biases), network_->layers[i]->layer_activation));
                z.push_back(forward_pass(a[i - 1], network_->layers[i]->weights, network_->layers[i]->biases));
            }
        }
        a.insert(a.begin(), {network_->input});
        return a;
    }

    std::vector<std::vector<std::vector<float>>> feedforward_z(network *network_)
    {
        std::vector<std::vector<std::vector<float>>> a;
        std::vector<std::vector<std::vector<float>>> z;
        for (int i = 0; i < network_->layers.size(); i++)
        {
            if (i == 0)
            {
                a.push_back(activate(forward_pass(network_->input, network_->layers[i]->weights, network_->layers[i]->biases), network_->layers[i]->layer_activation));
                z.push_back(forward_pass(network_->input, network_->layers[i]->weights, network_->layers[i]->biases));
            }
            else if (i != 0)
            {
                a.push_back(activate(forward_pass(a[i - 1], network_->layers[i]->weights, network_->layers[i]->biases), network_->layers[i]->layer_activation));
                z.push_back(forward_pass(a[i - 1], network_->layers[i]->weights, network_->layers[i]->biases));
            }
        }
        z.insert(z.begin(), {network_->input});
        return z;
    }

    std::vector<std::vector<float>> feedforward(std::vector<std::vector<float>> input_alt, network *network_)
    {
        std::vector<std::vector<std::vector<float>>> a;
        std::vector<std::vector<std::vector<float>>> z;
        for (int i = 0; i < network_->layers.size(); i++)
        {
            if (i == 0)
            {
                a.push_back(activate(forward_pass(input_alt, network_->layers[i]->weights, network_->layers[i]->biases), network_->layers[i]->layer_activation));
                z.push_back(forward_pass(input_alt, network_->layers[i]->weights, network_->layers[i]->biases));
            }
            else if (i != 0)
            {
                a.push_back(activate(forward_pass(a[i - 1], network_->layers[i]->weights, network_->layers[i]->biases), network_->layers[i]->layer_activation));
                z.push_back(forward_pass(a[i - 1], network_->layers[i]->weights, network_->layers[i]->biases));
            }
        }
        return a[a.size() - 1];
    }

    std::vector<std::vector<std::vector<float>>> feedforward_a(std::vector<std::vector<float>> input_alt, network *network_)
    {
        std::vector<std::vector<std::vector<float>>> a;
        std::vector<std::vector<std::vector<float>>> z;
        for (int i = 0; i < network_->layers.size(); i++)
        {
            if (i == 0)
            {
                a.push_back(activate(forward_pass(input_alt, network_->layers[i]->weights, network_->layers[i]->biases), network_->layers[i]->layer_activation));
                z.push_back(forward_pass(input_alt, network_->layers[i]->weights, network_->layers[i]->biases));
            }
            else if (i != 0)
            {
                a.push_back(activate(forward_pass(a[i - 1], network_->layers[i]->weights, network_->layers[i]->biases), network_->layers[i]->layer_activation));
                z.push_back(forward_pass(a[i - 1], network_->layers[i]->weights, network_->layers[i]->biases));
            }
        }
        a.insert(a.begin(), {input_alt});
        return a;
    }

    std::vector<std::vector<std::vector<float>>> feedforward_z(std::vector<std::vector<float>> input_alt, network *network_)
    {
        std::vector<std::vector<std::vector<float>>> a;
        std::vector<std::vector<std::vector<float>>> z;
        for (int i = 0; i < network_->layers.size(); i++)
        {
            if (i == 0)
            {
                a.push_back(activate(forward_pass(input_alt, network_->layers[i]->weights, network_->layers[i]->biases), network_->layers[i]->layer_activation));
                z.push_back(forward_pass(input_alt, network_->layers[i]->weights, network_->layers[i]->biases));
            }
            else if (i != 0)
            {
                a.push_back(activate(forward_pass(a[i - 1], network_->layers[i]->weights, network_->layers[i]->biases), network_->layers[i]->layer_activation));
                z.push_back(forward_pass(a[i - 1], network_->layers[i]->weights, network_->layers[i]->biases));
            }
        }
        z.insert(z.begin(), {input_alt});
        return z;
    }

    std::vector<std::vector<std::vector<std::vector<float>>>> backward_pass(network *network_, std::vector<std::vector<float>> output, std::vector<std::vector<float>> target)
    {
        std::vector<std::vector<std::vector<float>>> layerErrorTerms(network_->layers.size(), std::vector<std::vector<float>>(1, std::vector<float>(1, 0)));
        std::vector<std::vector<std::vector<float>>> layerGradientWeights(network_->layers.size(), std::vector<std::vector<float>>(1, std::vector<float>(1, 0)));
        std::vector<std::vector<std::vector<float>>> layerGradientBiases(network_->layers.size(), std::vector<std::vector<float>>(1, std::vector<float>(1, 0)));

        // Calculate activations during forward pass
        std::vector<std::vector<std::vector<float>>> calculated_a = feedforward_a(network_);

        for (int i = network_->layers.size() - 1; i > -1; i--)
        {
            if (i == network_->layers.size() - 1) // Output layer
            {
                if (network_->layers[i]->layer_activation == Softmax)
                {
                    layerErrorTerms[i] = matrix.difference(output, target); // Softmax specific
                }
                else
                {
                    layerErrorTerms[i] = matrix.multiply_hadamard(matrix.difference(output, target), activate_derivative(output, network_->layers[i]->layer_activation));
                }

                // Gradient weights: Error term times previous activation
                layerGradientWeights[i] = matrix.multiply(layerErrorTerms[i], matrix.transpose(calculated_a[i + 1]));
                layerGradientBiases[i] = layerErrorTerms[i]; // Gradient for biases is the same as error terms
            }
            else // Hidden layers
            {
                layerErrorTerms[i] = matrix.multiply_hadamard(matrix.multiply(matrix.transpose(network_->layers[i + 1]->weights), layerErrorTerms[i + 1]), activate_derivative(calculated_a[i + 1], network_->layers[i + 1]->layer_activation));

                // Gradient weights: Error term times previous activation
                layerGradientWeights[i] = matrix.multiply(layerErrorTerms[i], matrix.transpose(calculated_a[i]));
                layerGradientBiases[i] = layerErrorTerms[i];
            }
        }

        return {layerErrorTerms, layerGradientWeights, layerGradientBiases};
    }

    std::vector<std::vector<std::vector<std::vector<float>>>> backward_pass(std::vector<std::vector<float>> input_alt, network *network_, std::vector<std::vector<float>> output, std::vector<std::vector<float>> target)
    {
        std::vector<std::vector<std::vector<float>>> layerErrorTerms(network_->layers.size(), std::vector<std::vector<float>>(1, std::vector<float>(1, 0)));
        std::vector<std::vector<std::vector<float>>> layerGradientWeights(network_->layers.size(), std::vector<std::vector<float>>(1, std::vector<float>(1, 0)));
        std::vector<std::vector<std::vector<float>>> layerGradientBiases(network_->layers.size(), std::vector<std::vector<float>>(1, std::vector<float>(1, 0)));

        // Calculate activations during forward pass with alternative input
        std::vector<std::vector<std::vector<float>>> calculated_a = feedforward_a(input_alt, network_);

        for (int i = network_->layers.size() - 1; i > -1; i--)
        {
            if (i == network_->layers.size() - 1) // Output layer
            {
                if (network_->layers[i]->layer_activation == Softmax)
                {
                    layerErrorTerms[i] = matrix.difference(output, target); // Softmax specific
                }
                else
                {
                    layerErrorTerms[i] = matrix.multiply_hadamard(matrix.difference(output, target), activate_derivative(output, network_->layers[i]->layer_activation));
                }

                // Gradient weights: Error term times previous activation
                layerGradientWeights[i] = matrix.multiply(layerErrorTerms[i], matrix.transpose(calculated_a[i + 1]));
                layerGradientBiases[i] = layerErrorTerms[i]; // Gradient for biases is the same as error terms
            }
            else // Hidden layers
            {
                layerErrorTerms[i] = matrix.multiply_hadamard(matrix.multiply(matrix.transpose(network_->layers[i + 1]->weights), layerErrorTerms[i + 1]), activate_derivative(calculated_a[i + 1], network_->layers[i + 1]->layer_activation));

                // Gradient weights: Error term times previous activation
                layerGradientWeights[i] = matrix.multiply(layerErrorTerms[i], matrix.transpose(calculated_a[i]));
                layerGradientBiases[i] = layerErrorTerms[i];
            }
        }

        return {layerErrorTerms, layerGradientWeights, layerGradientBiases};
    }

    network *backpropagate(network *network_, std::vector<std::vector<float>> output, std::vector<std::vector<float>> target, float _n_)
    {
        std::vector<std::vector<std::vector<float>>> calculated_a = feedforward_a(network_);

        std::vector<std::vector<std::vector<std::vector<float>>>> backwardPass = backward_pass(network_, output, target);
        std::vector<std::vector<std::vector<float>>> layerErrorTerms = backwardPass[0];
        std::vector<std::vector<std::vector<float>>> layerGradientWeights = backwardPass[1];
        std::vector<std::vector<std::vector<float>>> layerGradientBiases = backwardPass[2];

        for (int i = network_->layers.size() - 1; i > -1; i--)
        {
            network_->layers[i]->update_weights(matrix.difference(network_->layers[i]->weights, matrix.multiply(_n_, matrix.multiply(layerErrorTerms[i], matrix.transpose(calculated_a[i])))));
            network_->layers[i]->update_biases(matrix.difference(network_->layers[i]->biases, matrix.multiply(_n_, layerErrorTerms[i])));
        }

        return network_;
    }

    network *backpropagate_batch(network *network_, std::vector<std::vector<std::vector<float>>> batch_inputs, std::vector<std::vector<std::vector<float>>> batch_targets, float _n_)
    {
        std::vector<std::vector<std::vector<float>>> accumulated_layerErrorTerms(network_->layers.size());
        std::vector<std::vector<std::vector<float>>> accumulated_layerGradientWeights(network_->layers.size());
        std::vector<std::vector<std::vector<float>>> accumulated_layerGradientBiases(network_->layers.size());

        int batch_size = batch_inputs.size();

        for (int i = 0; i < network_->layers.size(); i++)
        {
            accumulated_layerErrorTerms[i] = matrix.zeros(network_->layers[i]->weights);
            accumulated_layerGradientWeights[i] = matrix.zeros(network_->layers[i]->weights);
            accumulated_layerGradientBiases[i] = matrix.zeros(network_->layers[i]->biases);
        }

        for (int b = 0; b < batch_size; b++)
        {
            std::vector<std::vector<float>> output = feedforward(batch_inputs[b], network_);
            std::vector<std::vector<float>> target = batch_targets[b];

            std::vector<std::vector<std::vector<float>>> calculated_a = feedforward_a(batch_inputs[b], network_);

            std::vector<std::vector<std::vector<std::vector<float>>>> backwardPass = backward_pass(batch_inputs[b], network_, output, target);
            std::vector<std::vector<std::vector<float>>> layerErrorTerms = backwardPass[0];
            std::vector<std::vector<std::vector<float>>> layerGradientWeights = backwardPass[1];
            std::vector<std::vector<std::vector<float>>> layerGradientBiases = backwardPass[2];

            for (int a = 0; a < network_->layers.size(); a++)
            {
                accumulated_layerErrorTerms[a] = matrix.sum(accumulated_layerErrorTerms[a], layerErrorTerms[a]);
                accumulated_layerGradientWeights[a] = matrix.sum(accumulated_layerGradientWeights[a], layerGradientWeights[a]);
                accumulated_layerGradientBiases[a] = matrix.sum(accumulated_layerGradientBiases[a], layerGradientBiases[a]);
            }
        }

        for (int i = network_->layers.size() - 1; i > -1; i--)
        {
            accumulated_layerGradientWeights[i] = matrix.multiply(1.0f / batch_size, accumulated_layerGradientWeights[i]);
            accumulated_layerGradientBiases[i] = matrix.multiply(1.0f / batch_size, accumulated_layerGradientBiases[i]);

            network_->layers[i]->update_weights(matrix.difference(network_->layers[i]->weights, matrix.multiply(_n_, accumulated_layerGradientWeights[i])));
            network_->layers[i]->update_biases(matrix.difference(network_->layers[i]->biases, matrix.multiply(_n_, accumulated_layerGradientBiases[i])));
        }

        return network_;
    }

    network *mutate(network *network_, float amount = 1)
    {
        for (int i = 0; i < network_->layers.size(); i++)
        {
            for (int j = 0; j < network_->layers[i]->weights.size(); j++)
            {
                for (int k = 0; k < network_->layers[i]->weights[0].size(); k++)
                {
                    network_->layers[i]->weights[j][k] = lerp(
                        network_->layers[i]->weights[j][k],
                        createWeights(1)[0],
                        amount);
                }
            }
            for (int j = 0; j < network_->layers[i]->biases.size(); j++)
            {
                for (int k = 0; k < network_->layers[i]->biases[0].size(); k++)
                {
                    network_->layers[i]->biases[j][k] = lerp(
                        network_->layers[i]->biases[j][k],
                        createWeights(1)[0],
                        amount);
                }
            }
        }

        return network_;
    }

    // Network V2 support'
    std::vector<std::vector<float>> feedforward(network_v2 *network_)
    {
        std::vector<std::vector<std::vector<float>>> a;
        std::vector<std::vector<std::vector<float>>> z;
        for (int i = 0; i < network_->layers.size(); i++)
        {
            if (i == 0)
            {
                a.push_back(activate(forward_pass(network_->input, network_->layers[i]->weights, network_->layers[i]->biases), network_->layers[i]->layer_activation));
                z.push_back(forward_pass(network_->input, network_->layers[i]->weights, network_->layers[i]->biases));
            }
            else if (i != 0)
            {
                a.push_back(activate(forward_pass(a[i - 1], network_->layers[i]->weights, network_->layers[i]->biases), network_->layers[i]->layer_activation));
                z.push_back(forward_pass(a[i - 1], network_->layers[i]->weights, network_->layers[i]->biases));
            }
        }
        return a[a.size() - 1];
    }

    std::vector<std::vector<std::vector<float>>> feedforward_a(network_v2 *network_)
    {
        std::vector<std::vector<std::vector<float>>> a;
        std::vector<std::vector<std::vector<float>>> z;
        for (int i = 0; i < network_->layers.size(); i++)
        {
            if (i == 0)
            {
                a.push_back(activate(forward_pass(network_->input, network_->layers[i]->weights, network_->layers[i]->biases), network_->layers[i]->layer_activation));
                z.push_back(forward_pass(network_->input, network_->layers[i]->weights, network_->layers[i]->biases));
            }
            else if (i != 0)
            {
                a.push_back(activate(forward_pass(a[i - 1], network_->layers[i]->weights, network_->layers[i]->biases), network_->layers[i]->layer_activation));
                z.push_back(forward_pass(a[i - 1], network_->layers[i]->weights, network_->layers[i]->biases));
            }
        }
        a.insert(a.begin(), {network_->input});
        return a;
    }

    std::vector<std::vector<std::vector<float>>> feedforward_z(network_v2 *network_)
    {
        std::vector<std::vector<std::vector<float>>> a;
        std::vector<std::vector<std::vector<float>>> z;
        for (int i = 0; i < network_->layers.size(); i++)
        {
            if (i == 0)
            {
                a.push_back(activate(forward_pass(network_->input, network_->layers[i]->weights, network_->layers[i]->biases), network_->layers[i]->layer_activation));
                z.push_back(forward_pass(network_->input, network_->layers[i]->weights, network_->layers[i]->biases));
            }
            else if (i != 0)
            {
                a.push_back(activate(forward_pass(a[i - 1], network_->layers[i]->weights, network_->layers[i]->biases), network_->layers[i]->layer_activation));
                z.push_back(forward_pass(a[i - 1], network_->layers[i]->weights, network_->layers[i]->biases));
            }
        }
        z.insert(z.begin(), {network_->input});
        return z;
    }

    std::vector<std::vector<float>> feedforward(std::vector<std::vector<float>> input_alt, network_v2 *network_)
    {
        std::vector<std::vector<std::vector<float>>> a;
        std::vector<std::vector<std::vector<float>>> z;
        for (int i = 0; i < network_->layers.size(); i++)
        {
            if (i == 0)
            {
                a.push_back(activate(forward_pass(input_alt, network_->layers[i]->weights, network_->layers[i]->biases), network_->layers[i]->layer_activation));
                z.push_back(forward_pass(input_alt, network_->layers[i]->weights, network_->layers[i]->biases));
            }
            else if (i != 0)
            {
                a.push_back(activate(forward_pass(a[i - 1], network_->layers[i]->weights, network_->layers[i]->biases), network_->layers[i]->layer_activation));
                z.push_back(forward_pass(a[i - 1], network_->layers[i]->weights, network_->layers[i]->biases));
            }
        }
        return a[a.size() - 1];
    }

    std::vector<std::vector<std::vector<float>>> feedforward_a(std::vector<std::vector<float>> input_alt, network_v2 *network_)
    {
        std::vector<std::vector<std::vector<float>>> a;
        std::vector<std::vector<std::vector<float>>> z;
        for (int i = 0; i < network_->layers.size(); i++)
        {
            if (i == 0)
            {
                a.push_back(activate(forward_pass(input_alt, network_->layers[i]->weights, network_->layers[i]->biases), network_->layers[i]->layer_activation));
                z.push_back(forward_pass(input_alt, network_->layers[i]->weights, network_->layers[i]->biases));
            }
            else if (i != 0)
            {
                a.push_back(activate(forward_pass(a[i - 1], network_->layers[i]->weights, network_->layers[i]->biases), network_->layers[i]->layer_activation));
                z.push_back(forward_pass(a[i - 1], network_->layers[i]->weights, network_->layers[i]->biases));
            }
        }
        a.insert(a.begin(), {input_alt});
        return a;
    }

    std::vector<std::vector<std::vector<float>>> feedforward_z(std::vector<std::vector<float>> input_alt, network_v2 *network_)
    {
        std::vector<std::vector<std::vector<float>>> a;
        std::vector<std::vector<std::vector<float>>> z;
        for (int i = 0; i < network_->layers.size(); i++)
        {
            if (i == 0)
            {
                a.push_back(activate(forward_pass(input_alt, network_->layers[i]->weights, network_->layers[i]->biases), network_->layers[i]->layer_activation));
                z.push_back(forward_pass(input_alt, network_->layers[i]->weights, network_->layers[i]->biases));
            }
            else if (i != 0)
            {
                a.push_back(activate(forward_pass(a[i - 1], network_->layers[i]->weights, network_->layers[i]->biases), network_->layers[i]->layer_activation));
                z.push_back(forward_pass(a[i - 1], network_->layers[i]->weights, network_->layers[i]->biases));
            }
        }
        z.insert(z.begin(), {input_alt});
        return z;
    }

    std::vector<std::vector<std::vector<std::vector<float>>>> backward_pass(network_v2 *network_, std::vector<std::vector<float>> output, std::vector<std::vector<float>> target)
    {
        std::vector<std::vector<std::vector<float>>> layerErrorTerms(network_->layers.size(), std::vector<std::vector<float>>(1, std::vector<float>(1, 0)));
        std::vector<std::vector<std::vector<float>>> layerGradientWeights(network_->layers.size(), std::vector<std::vector<float>>(1, std::vector<float>(1, 0)));
        std::vector<std::vector<std::vector<float>>> layerGradientBiases(network_->layers.size(), std::vector<std::vector<float>>(1, std::vector<float>(1, 0)));

        // Calculate activations during forward pass
        std::vector<std::vector<std::vector<float>>> calculated_a = feedforward_a(network_);

        for (int i = network_->layers.size() - 1; i > -1; i--)
        {
            if (i == network_->layers.size() - 1) // Output layer
            {
                if (network_->layers[i]->layer_activation == Softmax)
                {
                    layerErrorTerms[i] = matrix.difference(output, target); // Softmax specific
                }
                else
                {
                    layerErrorTerms[i] = matrix.multiply_hadamard(matrix.difference(output, target), activate_derivative(output, network_->layers[i]->layer_activation));
                }

                // Gradient weights: Error term times previous activation
                layerGradientWeights[i] = matrix.multiply(layerErrorTerms[i], matrix.transpose(calculated_a[i + 1]));
                layerGradientBiases[i] = layerErrorTerms[i]; // Gradient for biases is the same as error terms
            }
            else // Hidden layers
            {
                layerErrorTerms[i] = matrix.multiply_hadamard(matrix.multiply(matrix.transpose(network_->layers[i + 1]->weights), layerErrorTerms[i + 1]), activate_derivative(calculated_a[i + 1], network_->layers[i + 1]->layer_activation));

                // Gradient weights: Error term times previous activation
                layerGradientWeights[i] = matrix.multiply(layerErrorTerms[i], matrix.transpose(calculated_a[i]));
                layerGradientBiases[i] = layerErrorTerms[i];
            }
        }

        return {layerErrorTerms, layerGradientWeights, layerGradientBiases};
    }

    std::vector<std::vector<std::vector<std::vector<float>>>> backward_pass(std::vector<std::vector<float>> input_alt, network_v2 *network_, std::vector<std::vector<float>> output, std::vector<std::vector<float>> target)
    {
        std::vector<std::vector<std::vector<float>>> layerErrorTerms(network_->layers.size(), std::vector<std::vector<float>>(1, std::vector<float>(1, 0)));
        std::vector<std::vector<std::vector<float>>> layerGradientWeights(network_->layers.size(), std::vector<std::vector<float>>(1, std::vector<float>(1, 0)));
        std::vector<std::vector<std::vector<float>>> layerGradientBiases(network_->layers.size(), std::vector<std::vector<float>>(1, std::vector<float>(1, 0)));

        // Calculate activations during forward pass with alternative input
        std::vector<std::vector<std::vector<float>>> calculated_a = feedforward_a(input_alt, network_);

        for (int i = network_->layers.size() - 1; i > -1; i--)
        {
            if (i == network_->layers.size() - 1) // Output layer
            {
                if (network_->layers[i]->layer_activation == Softmax)
                {
                    layerErrorTerms[i] = matrix.difference(output, target); // Softmax specific
                }
                else
                {
                    layerErrorTerms[i] = matrix.multiply_hadamard(matrix.difference(output, target), activate_derivative(output, network_->layers[i]->layer_activation));
                }

                // Gradient weights: Error term times previous activation
                layerGradientWeights[i] = matrix.multiply(layerErrorTerms[i], matrix.transpose(calculated_a[i + 1]));
                layerGradientBiases[i] = layerErrorTerms[i]; // Gradient for biases is the same as error terms
            }
            else // Hidden layers
            {
                layerErrorTerms[i] = matrix.multiply_hadamard(matrix.multiply(matrix.transpose(network_->layers[i + 1]->weights), layerErrorTerms[i + 1]), activate_derivative(calculated_a[i + 1], network_->layers[i + 1]->layer_activation));

                // Gradient weights: Error term times previous activation
                layerGradientWeights[i] = matrix.multiply(layerErrorTerms[i], matrix.transpose(calculated_a[i]));
                layerGradientBiases[i] = layerErrorTerms[i];
            }
        }

        return {layerErrorTerms, layerGradientWeights, layerGradientBiases};
    }

    network_v2 *backpropagate(network_v2 *network_, std::vector<std::vector<float>> output, std::vector<std::vector<float>> target, float _n_)
    {
        std::vector<std::vector<std::vector<float>>> calculated_a = feedforward_a(network_);

        std::vector<std::vector<std::vector<std::vector<float>>>> backwardPass = backward_pass(network_, output, target);
        std::vector<std::vector<std::vector<float>>> layerErrorTerms = backwardPass[0];
        std::vector<std::vector<std::vector<float>>> layerGradientWeights = backwardPass[1];
        std::vector<std::vector<std::vector<float>>> layerGradientBiases = backwardPass[2];

        for (int i = network_->layers.size() - 1; i > -1; i--)
        {
            network_->layers[i]->update_weights(matrix.difference(network_->layers[i]->weights, matrix.multiply(_n_, matrix.multiply(layerErrorTerms[i], matrix.transpose(calculated_a[i])))));
            network_->layers[i]->update_biases(matrix.difference(network_->layers[i]->biases, matrix.multiply(_n_, layerErrorTerms[i])));
        }

        return network_;
    }

    network_v2 *backpropagate_batch(network_v2 *network_, std::vector<std::vector<std::vector<float>>> batch_inputs, std::vector<std::vector<std::vector<float>>> batch_targets, float _n_)
    {
        std::vector<std::vector<std::vector<float>>> accumulated_layerErrorTerms(network_->layers.size());
        std::vector<std::vector<std::vector<float>>> accumulated_layerGradientWeights(network_->layers.size());
        std::vector<std::vector<std::vector<float>>> accumulated_layerGradientBiases(network_->layers.size());

        int batch_size = batch_inputs.size();

        for (int i = 0; i < network_->layers.size(); i++)
        {
            accumulated_layerErrorTerms[i] = matrix.zeros(network_->layers[i]->weights);
            accumulated_layerGradientWeights[i] = matrix.zeros(network_->layers[i]->weights);
            accumulated_layerGradientBiases[i] = matrix.zeros(network_->layers[i]->biases);
        }

        for (int b = 0; b < batch_size; b++)
        {
            std::vector<std::vector<float>> output = feedforward(batch_inputs[b], network_);
            std::vector<std::vector<float>> target = batch_targets[b];

            std::vector<std::vector<std::vector<float>>> calculated_a = feedforward_a(batch_inputs[b], network_);

            std::vector<std::vector<std::vector<std::vector<float>>>> backwardPass = backward_pass(batch_inputs[b], network_, output, target);
            std::vector<std::vector<std::vector<float>>> layerErrorTerms = backwardPass[0];
            std::vector<std::vector<std::vector<float>>> layerGradientWeights = backwardPass[1];
            std::vector<std::vector<std::vector<float>>> layerGradientBiases = backwardPass[2];

            for (int a = 0; a < network_->layers.size(); a++)
            {
                accumulated_layerErrorTerms[a] = matrix.sum(accumulated_layerErrorTerms[a], layerErrorTerms[a]);
                accumulated_layerGradientWeights[a] = matrix.sum(accumulated_layerGradientWeights[a], layerGradientWeights[a]);
                accumulated_layerGradientBiases[a] = matrix.sum(accumulated_layerGradientBiases[a], layerGradientBiases[a]);
            }
        }

        for (int i = network_->layers.size() - 1; i > -1; i--)
        {
            accumulated_layerGradientWeights[i] = matrix.multiply(1.0f / batch_size, accumulated_layerGradientWeights[i]);
            accumulated_layerGradientBiases[i] = matrix.multiply(1.0f / batch_size, accumulated_layerGradientBiases[i]);

            network_->layers[i]->update_weights(matrix.difference(network_->layers[i]->weights, matrix.multiply(_n_, accumulated_layerGradientWeights[i])));
            network_->layers[i]->update_biases(matrix.difference(network_->layers[i]->biases, matrix.multiply(_n_, accumulated_layerGradientBiases[i])));
        }

        return network_;
    }

    network_v2 *mutate(network_v2 *network_, float amount = 1)
    {
        for (int i = 0; i < network_->layers.size(); i++)
        {
            for (int j = 0; j < network_->layers[i]->weights.size(); j++)
            {
                for (int k = 0; k < network_->layers[i]->weights[0].size(); k++)
                {
                    network_->layers[i]->weights[j][k] = lerp(
                        network_->layers[i]->weights[j][k],
                        createWeights(1)[0],
                        amount);
                }
            }
            for (int j = 0; j < network_->layers[i]->biases.size(); j++)
            {
                for (int k = 0; k < network_->layers[i]->biases[0].size(); k++)
                {
                    network_->layers[i]->biases[j][k] = lerp(
                        network_->layers[i]->biases[j][k],
                        createWeights(1)[0],
                        amount);
                }
            }
        }

        return network_;
    }
};

#endif