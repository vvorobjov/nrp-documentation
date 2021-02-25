#!groovy
// Load shared library at master branch
// the path to the repo with this library should be specified in Jenkins
// https://tomd.xyz/jenkins-shared-library/
// https://www.jenkins.io/doc/book/pipeline/shared-libraries/
@Library('nrp-shared-libs@master') _

def docs_version
pipeline
{
    environment
    {
        GAZEBO_ROS_DIR = "GazeboRosPackages"
        EXP_CONTROL_DIR = "ExperimentControl"
        BRAIN_SIMULATION_DIR = "BrainSimulation"
        CLE_DIR = "CLE"
        EXDBACKEND_DIR = "ExDBackend"
        VC_DIR = "VirtualCoach"
        ADMIN_SCRIPTS_DIR = "admin-scripts"
        USER_SCRIPTS_DIR = "user-scripts"
        DOCS_DIR = "nrp-documentation"
        GIT_CHECKOUT_DIR = "${env.DOCS_DIR}"

        // If parameter BRANCH_NAME is set, use it as topic,
        // otherwise, use env.BRANCH_NAME or env.CHANGE_BRANCH (if it's PR)
        TOPIC_BRANCH = selectTopicBranch(selectTopicBranch(env.BRANCH_NAME, env.CHANGE_BRANCH), params.BRANCH_NAME)

        // If parameter BASE_BRANCH_NAME is set, use it as default branch
        // otherwise, use development
        DEFAULT_BRANCH = selectTopicBranch('development', params.BASE_BRANCH_NAME)

        // If parameter ADMIN_SCRIPT_BRANCH is set, use it to try to checkout admin-scripts
        // otherwise, try to checkout TOPIC_BRANCH
        ADMIN_SCRIPT_BRANCH = selectTopicBranch(env.TOPIC_BRANCH, params.ADMIN_SCRIPT_BRANCH)
    }
    agent {
        docker {
            label 'master'
            // NEXUS_REGISTRY_IP and NEXUS_REGISTRY_PORT are Jenkins global variables
            image "${env.NEXUS_REGISTRY_IP}:${env.NEXUS_REGISTRY_PORT}/nrp:master"
            registryUrl "http://${env.NEXUS_REGISTRY_IP}:${env.NEXUS_REGISTRY_PORT}"
            registryCredentialsId 'nexusadmin'
            args '--entrypoint="" -u root --privileged'
        }
    }
    options { 
        // Skip code checkout prior running pipeline (only Jenkinsfile is checked out)
        skipDefaultCheckout true

        ansiColor('xterm')
    }

    stages
    {
        stage('Cloning code')
        {
            when
            {
                expression { true }
            }
            steps{
                sh "rm -rf *"

                sh "echo TOPIC_BRANCH: ${env.TOPIC_BRANCH}"
                sh "echo DEFAULT_BRANCH: ${env.DEFAULT_BRANCH}"
                sh "echo ADMIN_SCRIPT_BRANCH: ${env.ADMIN_SCRIPT_BRANCH}"

                // Checkout main project to GIT_CHECKOUT_DIR
                dir(env.GIT_CHECKOUT_DIR) {
                    checkout scm
                }

                cloneRepoTopic(env.GAZEBO_ROS_DIR,          'git@bitbucket.org:hbpneurorobotics/gazeborospackages.git',   env.TOPIC_BRANCH, env.DEFAULT_BRANCH,     '${USER}') 
                
                cloneRepoTopic(env.BRAIN_SIMULATION_DIR,    'git@bitbucket.org:hbpneurorobotics/brainsimulation.git',     env.TOPIC_BRANCH, env.DEFAULT_BRANCH,     '${USER}')
                cloneRepoTopic(env.EXDBACKEND_DIR,          'git@bitbucket.org:hbpneurorobotics/exdbackend.git',          env.TOPIC_BRANCH, env.DEFAULT_BRANCH,     '${USER}')
                cloneRepoTopic(env.EXP_CONTROL_DIR,         'git@bitbucket.org:hbpneurorobotics/experimentcontrol.git',   env.TOPIC_BRANCH, env.DEFAULT_BRANCH,     '${USER}')
                cloneRepoTopic(env.CLE_DIR,                 'git@bitbucket.org:hbpneurorobotics/cle.git',                 env.TOPIC_BRANCH, env.DEFAULT_BRANCH,     '${USER}')
                cloneRepoTopic(env.VC_DIR,                  'git@bitbucket.org:hbpneurorobotics/virtualcoach.git',        env.TOPIC_BRANCH, env.DEFAULT_BRANCH,     '${USER}')

                cloneRepoTopic(env.USER_SCRIPTS_DIR,        'git@bitbucket.org:hbpneurorobotics/user-scripts.git',        env.TOPIC_BRANCH, env.DEFAULT_BRANCH,     '${USER}')
                cloneRepoTopic(env.ADMIN_SCRIPTS_DIR,       'git@bitbucket.org:hbpneurorobotics/admin-scripts.git',       env.ADMIN_SCRIPT_BRANCH, 'master',       '${USER}')
                
            }
        }
        stage('Gathering Docs')
        {
          when
          {
                expression {  true }
          }
            steps
            {
                dir(env.GIT_CHECKOUT_DIR)
                {
                  sh "bash ./.ci/build.bash"
                  recordIssues enabledForFailure: true, tools: [sphinxBuild(pattern: 'sphinx_w.txt')], qualityGates: [[threshold: 1, type: 'TOTAL', unstable: true]]
                }
            }
        }
        stage('Deploy with ansible')
        {
            when
            {
                expression { return params.DEPLOY }
            }
            steps
            {
                script
                {
                    docs_version = readFile "version"
                    dir(env.DOCS_DIR)
                    {
                        
                        withCredentials([usernamePassword(credentialsId: 'website_ansible', usernameVariable: 'USER', passwordVariable: 'PASSWORD')])
                        {
                            ansiblePlaybook(credentialsId: "website_ansible_key", \
                                            colorized: true, \
                                            inventory: 'ansible/hosts', \
                                            playbook: 'ansible/deploy_docs.yml',  \
                                            become : true ,  \
                                            extraVars: [ansible_become_pass :  '$PASSWORD' , \
                                                        docs_version : "${docs_version}", \
                                                        sphinx_build_html :  '_build/html/' ] )}
                    }
                }
            }
        }
     }

}