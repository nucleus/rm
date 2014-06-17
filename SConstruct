compileTestsFlag = ARGUMENTS.get('test', '')
compileTests = (compileTestsFlag != '')

debugFlag = ARGUMENTS.get('debug', '')
debug = (debugFlag != '')

########################################################################################################

includeDirectories = ['.']
libraryDirectories = ['/usr/lib', '/usr/X11R6/lib']

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

env = Environment(CC = 'g++', CCFLAGS = ccflags, LINKFLAGS = linkflags, CPPPATH = includeDirectories)

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

libutil = env.Library('util', source = utilSourceFiles, LIBPATH = libraryDirectories)

sourceFiles = {	'rm' : Split("""
						rm/implementation/GLWidget.cpp
						rm/implementation/MainWindow.cpp
						rm/implementation/main.cpp
						""")
}

mocHeaderFiles = {	'rm' : Split("""
							rm/interface/GLWidget.hpp
							rm/interface/MainWindow.hpp
							""")
}

########################################################################################################

mocs = {}
for exercise, headers in mocHeaderFiles.iteritems():
	mocs[exercise] = [env.Moc4(header) for header in headers]

if compileTests:
	utilitiesTestSources = ['util/test/UtilTest.cpp']
	env.Program(target = 'UtilityTest', source = utilitiesTestSources, LIBPATH = libraryDirectories, LIBS = libutil + testLibs)
	
	kdtreeTestSources = ['util/test/KDTreeTest.cpp']
	env.Program(target = 'KDTreeTest', source = kdtreeTestSources, LIBPATH = libraryDirectories, LIBS = libutil + testLibs)
else:
	for exercise in sourceFiles.keys():
		if len(sourceFiles[exercise]) == 0:
			continue
		name = 'cg2' + exercise
		env.Program(target = name, source = sourceFiles[exercise] + mocs[exercise], LIBPATH=libraryDirectories, LIBS=mainLibs + libutil)
