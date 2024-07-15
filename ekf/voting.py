class SensorVoting:
    def __init__(self, ranges: list, variances: list):
        self.ranges = ranges
        self.variances = variances

    def vote(self, data: list):
        assert len(data) == len(self.ranges)

        filtered_data_indices = []

        for i in range(len(data)):
            if not self._is_sensor_saturated(data[i], self.ranges[i]):
                filtered_data_indices.append(i)

        n = 0
        d = 0

        for i in filtered_data_indices:
            weight = 1 / self.variances[i]

            n += data[i] * weight
            d += weight

        result = n / d if d != 0 else 0
        totalVariance = 1 / d if d != 0 else 0

        return [result, totalVariance]

    def _is_sensor_saturated(self, value, range):
        return abs(value) >= range
