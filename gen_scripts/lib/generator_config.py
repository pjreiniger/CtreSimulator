class GeneratorConfig:

    def __init__(self, cci_class_name, cci_prefix, cci_wrapper_type,
                 jni_package, jni_class_name, java_class_name):
        self.cci_class_name = cci_class_name
        self.cci_prefix = cci_prefix
        self.cci_wrapper_type = cci_wrapper_type
        self.cci_overriden_function_bodies = {}

        self.jni_package = jni_package
        self.full_jni_package = jni_package
        self.jni_class_name = jni_class_name
        self.include_jni_in_method_name = False
        self.jni_sanitized_name = []
        self.cci_getter_overrides = []
        self.default_value_lookup = {}
        self.extra_jni_header = ""
        self.cpp_enum_types = []

        self.java_class_name = java_class_name
        
    def get_stripped_cci_class_name(self):
        raise Exception("Override me")
        
    def get_wrapper_class_name(self):
        raise Exception("Override me")
