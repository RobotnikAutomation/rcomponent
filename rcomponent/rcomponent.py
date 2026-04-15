class Rcomponent:

	def __init__(self, node):
		self.node = node
		self.logger = node.get_logger()

	def init_component(self):
		self.logger.info("RComponent: init_component()")

	def configure(self):
		self.logger.info("RComponent: configure()")

	def activate(self):
		self.logger.info("RComponent: activate()")

	def deactivate(self):
		self.logger.info("RComponent: deactivate()")

	def cleanup(self):
		self.logger.info("RComponent: cleanup()")
