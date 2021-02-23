#!groovy
// Load shared library at master branch
// the path to the repo with this library should be specified in Jenkins
// https://tomd.xyz/jenkins-shared-library/
// https://www.jenkins.io/doc/book/pipeline/shared-libraries/
@Library('nrp-shared-libs@master') _

def TOPIC_BRANCH = 'development'
def BASE_BRANCH_NAME = 'development'
def ADMIN_SCRIPT_BRANCH = 'master'

pipeline
{
    environment
    {
        EXP_CONTROL_DIR = "ExperimentControl"
        BRAIN_SIMULATION_DIR = "BrainSimulation"
        CLE_DIR = "CLE"
        EXDBACKEND_DIR = "ExDBackend"
        VC_DIR = "VirtualCoach"
        ADMIN_SCRIPTS_DIR = "admin-scripts"
        USER_SCRIPTS_DIR = "user-scripts"
        DOCS_DIR = "nrp-documentation"
        GIT_CHECKOUT_DIR = "${env.DOCS_DIR}"

    }
    agent {
        label 'master'
        docker {
            // NEXUS_REGISTRY_IP and NEXUS_REGISTRY_PORT are Jenkins global variables
            image "${env.NEXUS_REGISTRY_IP}:${env.NEXUS_REGISTRY_PORT}/nrp:master"
            args '--entrypoint="" -u root --privileged'
        }
    }
    options { 
        // Skip code checkout prior running pipeline (only Jenkinsfile is checked out)
        skipDefaultCheckout true
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
                script{
                    if (params.BRANCH_NAME) {
                        TOPIC_BRANCH = params.BRANCH_NAME
                    }
                    else {
                        TOPIC_BRANCH = selectTopicBranch(env.BRANCH_NAME, env.CHANGE_BRANCH)
                    }
                    if (params.BASE_BRANCH_NAME) {
                        BASE_BRANCH_NAME = params.BASE_BRANCH_NAME
                    }
                    if (params.ADMIN_SCRIPT_BRANCH) {
                        ADMIN_SCRIPT_BRANCH = params.ADMIN_SCRIPT_BRANCH
                    }
                    else {
                        ADMIN_SCRIPT_BRANCH = TOPIC_BRANCH
                    }
                }

                sh 'echo BRANCH_NAME: ${BRANCH_NAME}'
                sh 'echo BASE_BRANCH_NAME: ${BASE_BRANCH_NAME}'
                sh 'echo ADMIN_SCRIPT_BRANCH: ${ADMIN_SCRIPT_BRANCH}'

                // Checkout main project to GIT_CHECKOUT_DIR
                dir(env.GIT_CHECKOUT_DIR) {
                    checkout scm
                }

                cloneRepoTopic(env.BRAIN_SIMULATION_DIR,    'git@bitbucket.org:hbpneurorobotics/brainsimulation.git',     TOPIC_BRANCH, BASE_BRANCH_NAME,     '${USER}')
                cloneRepoTopic(env.EXDBACKEND_DIR,          'git@bitbucket.org:hbpneurorobotics/exdbackend.git',          TOPIC_BRANCH, BASE_BRANCH_NAME,     '${USER}')
                cloneRepoTopic(env.EXP_CONTROL_DIR,         'git@bitbucket.org:hbpneurorobotics/experimentcontrol.git',   TOPIC_BRANCH, BASE_BRANCH_NAME,     '${USER}')
                cloneRepoTopic(env.CLE_DIR,                 'git@bitbucket.org:hbpneurorobotics/cle.git',                 TOPIC_BRANCH, BASE_BRANCH_NAME,     '${USER}')
                cloneRepoTopic(env.VC_DIR,                  'git@bitbucket.org:hbpneurorobotics/virtualcoach.git',        TOPIC_BRANCH, BASE_BRANCH_NAME,     '${USER}')

                cloneRepoTopic(env.ADMIN_SCRIPTS_DIR,       'git@bitbucket.org:hbpneurorobotics/admin-scripts.git',       ADMIN_SCRIPT_BRANCH, 'master',       '${USER}')
                
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
                script
                {
                  sh '''!/bin/bash
                  export HBP=${PWD}
                  cd ${GIT_CHECKOUT_DIR}
                  make doc
                  '''
                }
            }
        }
        // stage('Push Docker Images to Nexus Registry')
        // {
        //     when
        //     {
        //         expression {  true }
        //     }
        //     steps
        //     {
        //         script
        //         {
        //             withCredentials([usernamePassword(credentialsId: 'nexusadmin', usernameVariable: 'USER', passwordVariable: 'PASSWORD')])
        //             {
        //                 sh 'docker login -u $USER -p $PASSWORD ${NexusDockerRegistryUrl}'
        //             }
        //             sh "docker tag nrp:dev ${NexusDockerRegistryUrl}/nrp:${branch_tag}"
        //             sh "docker tag hbpneurorobotics/nrp_frontend:dev ${NexusDockerRegistryUrl}/nrp_frontend:${branch_tag}"
        //             sh "docker push ${NexusDockerRegistryUrl}/nrp:${branch_tag}"
        //             sh "docker push ${NexusDockerRegistryUrl}/nrp_frontend:${branch_tag}"
        //             sh 'docker logout ${NexusDockerRegistryUrl}'
        //         }
        //     }
        // }
        // stage('Deploy with ansible')
        // {
        //     when
        //     {
        //         expression { return params.DEPLOY_IMAGE }
        //     }
        //     steps
        //     {
        //         script
        //         {

        //             dir('admin-scripts')
        //             {
        //                 git branch: "${params.ADMIN_SCRIPT_BRANCH}", url: 'git@bitbucket.org:hbpneurorobotics/admin-scripts.git'


        //                 withCredentials([usernamePassword(credentialsId: 'nexusadmin', usernameVariable: 'USER', passwordVariable: 'PASSWORD')])
        //                 {
        //                     //update backends first
        //                     ansiblePlaybook(credentialsId: 'test-key', inventory: 'ansible/hosts', playbook: 'ansible/update.yml', limit : "${params.env}_backends", become : true , extraVars: [docker_tag :  "${branch_tag}" , docker_reg : "${NexusDockerRegistryUrl}", docker_user :  '$USER', docker_pass :  '$PASSWORD' ] )
        //                     //keep_running backends
        //                     ansiblePlaybook(credentialsId: 'test-key', inventory: 'ansible/hosts', playbook: 'ansible/keep_running.yml', limit : "${params.env}_backends", become : true , extraVars: [docker_tag :  "${branch_tag}" , docker_reg : "${NexusDockerRegistryUrl}", force_config: "true" , jenkins_home : "${HOME}"] )

        //                     //update frontend
        //                     ansiblePlaybook(credentialsId: 'test-key', inventory: 'ansible/hosts', playbook: 'ansible/update.yml', limit : "${params.env}_frontend", become : true , extraVars: [docker_tag :  "${branch_tag}" , docker_reg : "${NexusDockerRegistryUrl}", docker_user :  '$USER', docker_pass :  '$PASSWORD' ] )
        //                     //keep_running frontend
        //                     ansiblePlaybook(credentialsId: 'test-key', inventory: 'ansible/hosts', playbook: 'ansible/keep_running.yml', limit : "${params.env}_frontend", become : true , extraVars: [docker_tag :  "${branch_tag}" , docker_reg : "${NexusDockerRegistryUrl}", force_config: "true" , jenkins_home : "${HOME}" ] )
        //                 }
        //             }
        //         }
        //     }
        // }
     }

}