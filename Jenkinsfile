pipeline {
    agent {
        label "rmc-vosl423-x8664-build01"
    }

    parameters {
        string(name: 'tox_args', defaultValue: '-v', description: 'Arguments passed to tox besides the environment name', )
        string(name: 'pytest_args', defaultValue: '-vx -m "(core or gui or share_elements) and not unstable and not user_input"', description: 'Arguments passed to pytest')
    }

    environment {
        PATH = "/home_local/l_buildb/.local/bin:$PATH"
        TOX_LIMITED_SHEBANG = 1
        // Allows 1st build of a project to succeed, workaround for https://issues.jenkins-ci.org/browse/JENKINS-41929
        tox_args = "${params.tox_args}"
        pytest_args = "${params.pytest_args}"
    }

    options {
        timestamps()
    }

    stages {

        stage('Prepare tox') {
            steps {
                sh 'printenv | sort'
                sh 'pip3 install -q --user --ignore-installed tox==3.12'
            }
        }

        stage('Test Python 2.7') {
            steps {
                timeout(time: 10, unit: 'MINUTES') {
                    // Run test
                    // * wrapped in xvfb-run for having an X server
                    // * specify tox environment
                    // * run only stable tests
                    // * collect pytest results in XML file
                    // * set absolute cache_dir
                    sh "xvfb-run -as '-screen 0 1920x1200x24' tox -e py27 $tox_args -- $pytest_args --junitxml $WORKSPACE/pytest_py27_results.xml -o cache_dir=$WORKSPACE |& tee pytestout.txt"
                }
            }
        }

        stage('Test Python 3.4') {
            steps {
                timeout(time: 10, unit: 'MINUTES') {
                    sh "xvfb-run -as '-screen 0 1920x1200x24' tox -e py34 $tox_args -- $pytest_args --junitxml $WORKSPACE/pytest_py34_results.xml -o cache_dir=$WORKSPACE |& tee pytestout.txt"
                }
            }
        }

        stage('Test Python 3.6') {
            environment {
                PATH = "/opt/python/osl42-x86_64/python3/stable/1.0.1/bin:$PATH"
                LD_LIBRARY_PATH = "/opt/python/osl42-x86_64/python3/stable/1.0.1/lib:$LD_LIBRARY_PATH"
            }
            steps {
                timestamps {
                    timeout(time: 10, unit: 'MINUTES') {
                        sh "xvfb-run -as '-screen 0 1920x1200x24' tox -e py36 $tox_args -- $pytest_args --junitxml $WORKSPACE/pytest_py36_results.xml -o cache_dir=$WORKSPACE |& tee pytestout.txt"
                    }
                }
            }
        }

        stage('Test Python 2.7 Coverage') {
            steps {
                timeout(time: 10, unit: 'MINUTES') {
                    sh "xvfb-run -as '-screen 0 1920x1200x24' tox -e coverage $tox_args -- $pytest_args -o cache_dir=$WORKSPACE"
                }
            }
        }

        stage('Build Documentation') {
            steps {
                sh 'xvfb-run -as "-screen 0 1920x1200x24" tox -e docs $tox_test_params'
                // sphinx linkcheck only generates relative files, which cannot be found by recordIssues
                // therefore, we need to make the path absolute
                sh 'sed -i "s#.*#$WORKSPACE/doc/&#" build_doc/output.txt'
                // Find warnings:
                // * in the sphinx build process
                // * in the sphinx linkcheck results
                // * in the pytest warnings section
                recordIssues filters: [excludeCategory('.*rafcon.*'), excludeCategory('redirected with Found')], tools: [sphinxBuild(), groovyScript(parserId: 'sphinx-linkcheck', pattern: 'build_doc/output.txt'), groovyScript(parserId: 'pytest', pattern: 'pytestout.txt')]
            }
        }

    }
    post {
        failure {
            rocketSend channel: 'rafcon-jenkins', avatar: 'https://rmc-jenkins.robotic.dlr.de/jenkins/static/ff676c77/images/headshot.png', message: ":sob: <$BUILD_URL|Build $BUILD_NUMBER> on branch '$BRANCH_NAME' *failed*! Commit: <https://rmc-github.robotic.dlr.de/common/rafcon/commit/$GIT_COMMIT|$GIT_COMMIT> :sob:", rawMessage: true
        }
        unstable {
            junit '**/pytest_*_results.xml'
            rocketSend channel: 'rafcon-jenkins', avatar: 'https://rmc-jenkins.robotic.dlr.de/jenkins/static/ff676c77/images/headshot.png', message: ":sob: <$BUILD_URL|Build $BUILD_NUMBER> on branch '$BRANCH_NAME' *failed* (unstable)! Commit: <https://rmc-github.robotic.dlr.de/common/rafcon/commit/$GIT_COMMIT|$GIT_COMMIT> :sob:", rawMessage: true
        }
        success {
            junit '**/pytest_*_results.xml'
            cobertura autoUpdateHealth: false, autoUpdateStability: false,
                coberturaReportFile: 'pytest-cov_results.xml',
                conditionalCoverageTargets: '70, 0, 0', lineCoverageTargets: '80, 0, 0', methodCoverageTargets: '80, 0, 0',
                maxNumberOfBuilds: 0,
                enableNewApi: true,
                failUnhealthy: false, failUnstable: false,
                sourceEncoding: 'ASCII', zoomCoverageChart: false
            publishHTML([allowMissing: false, alwaysLinkToLastBuild: false, keepAll: false, reportDir: 'build_doc', reportFiles: 'index.html', reportName: 'Documentation', reportTitles: 'RAFCON documentation'])
            rocketSend channel: 'rafcon-jenkins', avatar: 'https://rmc-jenkins.robotic.dlr.de/jenkins/static/ff676c77/images/headshot.png', message: ":tada: <$BUILD_URL|Build $BUILD_NUMBER> on branch '$BRANCH_NAME' *succeeded*! Commit: <https://rmc-github.robotic.dlr.de/common/rafcon/commit/$GIT_COMMIT|$GIT_COMMIT> :tada:", rawMessage: true
            archiveArtifacts artifacts: '.tox/dist/*', fingerprint: true
        }
    }
}
