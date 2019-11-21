from conans import ConanFile, CMake, tools


class UbitrackCoreConan(ConanFile):
    name = "ubitrack_device_camera_kinect4azure"
    version = "1.3.0"

    description = "Ubitrack Device Camera Kinect for Azure"
    url = "https://github.com/Ubitrack/device_camera_kinect4azure.git"
    license = "GPL"

    short_paths = True
    settings = "os", "compiler", "build_type", "arch"
    generators = "cmake"

    options = {
        "workspaceBuild" : [True, False],
    }

    default_options = {
        "workspaceBuild" : False,
        "ubitrack_core:shared" : True,
        "ubitrack_vision:shared" : True,
        "ubitrack_dataflow:shared" : True,
    }
        

    # all sources are deployed with the package
    exports_sources = "cmake/*", "doc/*", "src/*", "CMakeLists.txt"

    def requirements(self):
        userChannel = "ubitrack/stable"
        if self.options.workspaceBuild:
            userChannel = "local/dev"
        self.requires("ubitrack_core/%s@%s" % (self.version, userChannel))
        self.requires("ubitrack_vision/%s@%s" % (self.version, userChannel))
        self.requires("ubitrack_dataflow/%s@%s" % (self.version, userChannel))
        self.requires("kinect-azure-sensor-sdk/1.3.0@camposs/stable")
       

    def imports(self):
        self.copy(pattern="*.dll", dst="bin", src="bin") # From bin to bin
    #     self.copy(pattern="*.dylib*", dst="lib", src="lib") 
    #     self.copy(pattern="*.so*", dst="lib", src="lib") 
       
    def build(self):
        cmake = CMake(self)
        cmake.verbose = True
        cmake.definitions['WITH_OPENCL'] = self.options['ubitrack_vision'].with_opencl
        cmake.configure()
        cmake.build()
        cmake.install()

    def package(self):
        pass

    def package_info(self):
        pass
