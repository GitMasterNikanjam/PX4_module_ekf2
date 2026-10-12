/**
 * @file EKF2Selector.hpp
 * @brief Host-side selection policy for multiple EKF instances.
 */
#pragma once

#include <cmath>
#include <cstdint>

/**
 * @brief Selects among host-managed EKF instances.
 *
 * The selector does not inspect EKF internals. The host supplies each
 * instance's health, fault state, and aggregate error score, and owns the EKF
 * objects and their scheduling. Lower finite scores are preferred. A healthy
 * current instance is retained until a candidate has less than half its score
 * and the current selection has remained stable for at least one second.
 * An unhealthy/faulted current instance is replaced immediately. A valid
 * requested instance takes priority over score-based selection.
 *
 * Call update() regularly with a monotonic timestamp. The object stores only
 * the selected index and the time at which that selection began.
 */
class EKF2Selector
{
public:
	/** @brief Sentinel returned when no usable instance is selected. */
	static constexpr uint8_t INVALID_INSTANCE = UINT8_MAX; ///< Compile-time constant defining INVALID INSTANCE.
	/** @brief Minimum dwell time before score-based switching, in microseconds. */
	static constexpr uint64_t MIN_SELECTION_TIME_US = 1'000'000; ///< Compile-time constant defining MIN SELECTION TIME US.
	/** @brief Candidate score must be below this fraction of current score. */
	static constexpr float SCORE_SWITCH_RATIO = 0.5f; ///< Compile-time constant defining SCORE SWITCH RATIO.

	/** @brief Host-provided status and aggregate score for one EKF instance. */
	struct InstanceStatus {
		bool healthy{false}; ///< Whether the instance currently provides usable estimates.
		bool fault{false}; ///< Whether a detected fault disqualifies this instance.
		float error_score{NAN}; ///< Finite lower-is-better score; NaN/Inf excludes the instance as a candidate.
	};

	/**
	 * @brief Update selection using current instance status.
	 * @param instances Array of `count` status records; may be null only when count is zero.
	 * @param count Number of available instances.
	 * @param now_us Current monotonic time in microseconds.
	 * @param requested_instance Optional preferred index. A healthy, non-faulted
	 *        requested instance is selected immediately; use INVALID_INSTANCE
	 *        for automatic selection.
	 * @return Selected index, or INVALID_INSTANCE if no usable candidate exists.
	 *
	 * A null array or zero count clears the selection. If no healthy, non-faulted
	 * candidate has a finite score, the selection is also cleared. An explicitly
	 * requested healthy instance may be selected even when its score is not finite.
	 */
	/** @brief Re-evaluate requested and automatic candidates and return the selected instance. */
	uint8_t update(const InstanceStatus *instances, uint8_t count, uint64_t now_us,
		       uint8_t requested_instance = INVALID_INSTANCE)
	{
		if (instances == nullptr || count == 0) {
			_selected = INVALID_INSTANCE;
			return _selected;
		}

		if (requested_instance < count && instances[requested_instance].healthy && !instances[requested_instance].fault) {
			select(requested_instance, now_us);
			return _selected;
		}

		uint8_t best = INVALID_INSTANCE; ///< Stores best.

		for (uint8_t i = 0; i < count; ++i) {
			if (instances[i].healthy && !instances[i].fault && std::isfinite(instances[i].error_score)
			    && (best == INVALID_INSTANCE || instances[i].error_score < instances[best].error_score)) {
				best = i;
			}
		}

		if (best == INVALID_INSTANCE) {
			_selected = INVALID_INSTANCE;
			return _selected;
		}

		if (_selected >= count || !instances[_selected].healthy || instances[_selected].fault) {
			select(best, now_us);
			return _selected;
		}

		const float current_score = instances[_selected].error_score; ///< Stores current score.
		const bool sufficient_improvement = !std::isfinite(current_score)
					    || instances[best].error_score < current_score * SCORE_SWITCH_RATIO;
		const bool selection_is_stable = now_us >= _selected_since_us
					 && now_us - _selected_since_us >= MIN_SELECTION_TIME_US;

		if (best != _selected && sufficient_improvement && selection_is_stable) {
			select(best, now_us);
		}

		return _selected;
	}

	/** @brief Return the index selected by the latest update(), if any. */
	uint8_t selectedInstance() const { return _selected; }

private:
	/** @brief Store a new selected index and its selection start time. */
	void select(uint8_t instance, uint64_t now_us)
	{
		if (_selected != instance) {
			_selected = instance;
			_selected_since_us = now_us;
		}
	}

	uint8_t _selected{INVALID_INSTANCE}; ///< Stores selected.
	uint64_t _selected_since_us{0}; ///< Timestamp or duration value in microseconds: selected since us.
};
