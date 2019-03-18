pipeline {
    agent {
        label "rmc-vosl423-x8664-build01"
    }

    parameters {
        string(defaultValue: "-v", description: 'Additional parameters passed to tox when running the tests', name: 'tox_test_params')
    }


    environment {
        TOX_LIMITED_SHEBANG = 1
    }

    stages {

        stage('Prepare tox') {
            steps {
                timestamps {
                    sh 'printenv'
                    sh 'pip3 install --user --upgrade tox==3.7'
                }
            }
        }

        stage('Test Python 2.7') {
            steps {
                timestamps {
                    timeout(time: 10, unit: 'MINUTES') {
                        // Reset RAFCON config files
                        sh 'rm -f $HOME/.config/rafcon/*'
                        // Run test
                        // * wrapped in xvfb-run for having an X server
                        // * specify tox environment
                        // * run only stable tests
                        // * collect pytest results in XML file
                        // * set absolute cache_dir
                        sh 'xvfb-run -as "-screen 0 1920x1200x24" ~/.local/bin/tox -e py27 $tox_test_params -- -vx -m "(core or gui or share_elements) and not unstable" --junitxml $WORKSPACE/pytest_py27_results.xml -o cache_dir=$WORKSPACE'
                    }
                }
            }
        }

        stage('Test Python 3.4') {
            steps {
                timestamps {
                    timeout(time: 10, unit: 'MINUTES') {
                        sh 'rm -f $HOME/.config/rafcon/*'
                        sh 'xvfb-run -as "-screen 0 1920x1200x24" ~/.local/bin/tox -e py34 $tox_test_params -- -vx -m "(core or gui or share_elements) and not unstable" --junitxml $WORKSPACE/pytest_py34_results.xml -o cache_dir=$WORKSPACE'
                    }
                }
            }
        }

        stage('Build Documentation') {
            steps {
                timestamps {
                    sh 'xvfb-run -as "-screen 0 1920x1200x24" tox -e docs'
                    sh 'sed -i "s#.*#$WORKSPACE/doc/&#" build_doc/output.txt'
                    recordIssues filters: [excludeCategory('.*rafcon.*'), excludeCategory('redirected with Found')], tools: [sphinxBuild(), groovyScript(parserId: 'sphinx-linkcheck', pattern: 'build_doc/output.txt')]
                }
            }
        }

    }
    post {
        failure {
            rocketSend channel: 'rafcon-jenkins', avatar: 'https://rmc-jenkins.robotic.dlr.de/jenkins/static/ff676c77/images/headshot.png', message: ":sob: <$BUILD_URL|Build $BUILD_NUMBER> on branch '$BRANCH_NAME' *failed*! Commit: <https://rmc-github.robotic.dlr.de/common/rafcon/commit/$GIT_COMMIT|$GIT_COMMIT> :sob:", rawMessage: true
        }
        success {
            publishHTML([allowMissing: false, alwaysLinkToLastBuild: false, keepAll: false, reportDir: 'build_doc', reportFiles: 'index.html', reportName: 'Documentation', reportTitles: 'RAFCON documentation'])
            rocketSend channel: 'rafcon-jenkins', avatar: 'https://rmc-jenkins.robotic.dlr.de/jenkins/static/ff676c77/images/headshot.png', message: ":tada: <$BUILD_URL|Build $BUILD_NUMBER> on branch '$BRANCH_NAME' *succeeded*! Commit: <https://rmc-github.robotic.dlr.de/common/rafcon/commit/$GIT_COMMIT|$GIT_COMMIT> :tada:", rawMessage: true
        }
        always {
            timestamps {
                junit '**/pytest_*_results.xml'
                archiveArtifacts artifacts: '.tox/dist/*', fingerprint: true, onlyIfSuccessful: true
            }
        }
    }
}
