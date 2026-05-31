from holoscan.conditions import PeriodicCondition
from holoscan.core import Fragment

from steering_wheel_app import SteeringWheelOperator


class SteeringWheelFragment(Fragment):
	def compose(self):
		steering_op = SteeringWheelOperator(
			self,
			PeriodicCondition(self, recess_period=10_000_000),  # 10 ms → 100 Hz
			name="steering_wheel",
		)
		self.add_operator(steering_op)
