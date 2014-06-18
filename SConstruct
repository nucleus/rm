debugFlag = ARGUMENTS.get('debug', '')
debug = (debugFlag != '')

arch = ARGUMENTS.get('arch', '')
archFlag = '-arch='
if arch == '':
	archFlag += 'sm_20'
else:
	archFlag += arch

########################################################################################################

includeDirectories = ['.', '/usr/local/cuda/include']
libraryDirectories = ['/usr/lib', '/usr/X11R6/lib', '/usr/local/cuda/lib64', '/usr/local/cuda/lib']

commonLibs = ['GL', 'GLU']
testLibs = commonLibs + []
mainLibs = commonLibs + ['QtOpenGL', 'QtGui', 'QtCore']

ccflags = '-std=c++0x'
if debug:
	ccflags += ' -g'
else:
	ccflags += ' -O3 -fopenmp'

linkflags = ''
if not debug:
	linkflags += ' -fopenmp'

########################################################################################################
	
def MapSource(env, source):
	if source[-3:] == ".cu":
		return env.Cuda(source)
	return source
	
########################################################################################################

env = Environment(CC = 'g++', CCFLAGS = ccflags, LINKFLAGS = linkflags, CPPPATH = includeDirectories)

env.Append(BUILDERS = {'Cuda': Builder(
	action='/usr/local/cuda/bin/nvcc ' + archFlag + ' $SOURCE -c -o $TARGET -I.',
    suffix = '.o',
    src_suffix = '.cu'
)})

env['BUILD_ROOT'] = Dir('.')
env['QT4DIR'] = '/usr'
env['ENV']['PKG_CONFIG_PATH'] = '/usr/lib/qt4/pkg-config'

env.Tool('qt4')
env.EnableQt4Modules(['QtGui', 'QtCore', 'QtOpenGL'])

########################################################################################################

utilSourceFiles = Split("""
util/implementation/Util.cpp
util/implementation/BoundingBox.cpp
util/implementation/Grid3D.cpp
util/implementation/ImplicitSurface.cpp
""")

#libutil = env.Library('util', source = utilSourceFiles, LIBPATH = libraryDirectories)

sourceFiles = {	'rm' : Split("""
						rm/implementation/GLWidget.cpp
						rm/implementation/MainWindow.cpp
						rm/implementation/main.cpp
						""")
}

cudaSourceFiles = { 'rm' : Split("""
				util/implementation/ImplicitSurfaceKernels.cu
				""")
}

#cudaSourceFiles = { 'rm' : [] }

mocHeaderFiles = {	'rm' : Split("""
							rm/interface/GLWidget.hpp
							rm/interface/MainWindow.hpp
							""")
}

########################################################################################################

mocs = {}
for exercise, headers in mocHeaderFiles.iteritems():
	mocs[exercise] = [env.Moc4(header) for header in headers]

for exercise in sourceFiles.keys():
	if len(sourceFiles[exercise]) == 0:
		continue
	name = 'cg2' + exercise
	guiObjects = env.Object(source = sourceFiles[exercise])
	mocObjects = env.Object(source = mocs[exercise])	
	cudaObjects = [MapSource(env, src) for src in cudaSourceFiles[exercise]]
	libObjects = env.Object(source = utilSourceFiles)
	env.Program(target = name, source = guiObjects + mocObjects + libObjects + cudaObjects, LIBPATH=libraryDirectories, LIBS=mainLibs + ['cudart'])
