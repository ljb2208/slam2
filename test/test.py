import subprocess


ncc_base = 0.80
mf_base = 2

ncc_it = 20
mf_it = 5


def getResult():
    outputFile = open("test-output.txt", "r")
    lines = outputFile.readlines()

    for l in lines:
        if ("Average translation error:" not in l):
            continue
        
        pos = l.find(":")
        
        return l[pos + 1:].strip()

def plotResult(ncc, mf):
    fny = "output/ploty" + str(ncc) + "-" + str(mf) + ".png"
    fnxz = "output/plotpath" + str(ncc) + "-" + str(mf) + ".png"

    cmd = "set terminal png size 800,600; set output '" + fny + "'; load '../util/ploty.plt'"
    
    subprocess.call(["gnuplot", "-e", cmd])

    cmd = "set terminal png size 800,600; set output '" + fnxz + "'; load '../util/plotpath.plt'"
    
    subprocess.call(["gnuplot", "-e", cmd])


def createSettingsFile(ncc, mf):
    templateFile = open("test-settings-template.txt", "r")
    settingsFile = open("test-settings.txt", "w")
    settingsFile.write(templateFile.read())
    settingsFile.write("nccTolerance:" + str(ncc) + "\n")
    settingsFile.write("maxFeatures:" + str(mf) + "\n")
    templateFile.close()
    settingsFile.close()

def runTest(ncc, mf):
    createSettingsFile(ncc, mf)
    outputFile = open("test-output.txt", "w")
    subprocess.call(["../build/bin/slam_dataset", "settings=test-settings.txt"], stdout=outputFile)
    outputFile.close()
    result = getResult()
    resultFile.write(str(ncc) + "," + str(mf) + "," + result + "\n")
    plotResult(ncc, mf)
    print("Test completed: " + str(ncc) + " " + str(mf) + "\n")



resultFile = open("results.csv", "w+")
resultFile.write("NCC,MaxFeatures,Result\n")


for i in range(0, ncc_it):
    ncc = ncc_base + 0.01 * i
    for y in range(0, mf_it):
        mf = mf_base + 2*y
        print("ncc:" + str(ncc) + "mf:" + str(mf) + "\n")
        runTest(ncc, mf)

resultFile.close()