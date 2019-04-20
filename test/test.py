import subprocess


ncc_base = 0.87
mf_base = 10
fa_base = 0
ncc_it = 10
mf_it = 1
fa_it = 10


def getResult():
    outputFile = open("test-output.txt", "r")
    lines = outputFile.readlines()

    for l in lines:
        if ("Average translation error:" not in l):
            continue
        
        pos = l.find(":")
        
        return l[pos + 1:].strip()

def plotResult(ncc, mf, fa):
    fny = "output/ploty" + str(ncc) + "-" + str(mf) + "-" + str(fa) + ".png"
    fnxz = "output/plotpath" + str(ncc) + "-" + str(mf) + "-" + str(fa) + ".png"

    cmd = "set terminal png size 800,600; set output '" + fny + "'; load '../util/ploty.plt'"
    
    subprocess.call(["gnuplot", "-e", cmd])

    cmd = "set terminal png size 800,600; set output '" + fnxz + "'; load '../util/plotpath.plt'"
    
    subprocess.call(["gnuplot", "-e", cmd])


def createSettingsFile(ncc, mf, fa):
    templateFile = open("test-settings-template.txt", "r")
    settingsFile = open("test-settings.txt", "w")
    settingsFile.write(templateFile.read())
    settingsFile.write("nccTolerance:" + str(ncc) + "\n")
    settingsFile.write("maxFeatures:" + str(mf) + "\n")
    settingsFile.write("featureAgeDiscrim:" + str(fa) + "\n")
    templateFile.close()
    settingsFile.close()

def runTest(ncc, mf, fa):
    createSettingsFile(ncc, mf,fa)
    outputFile = open("test-output.txt", "w")
    subprocess.call(["../build/bin/slam_dataset", "settings=test-settings.txt"], stdout=outputFile)
    outputFile.close()
    result = getResult()
    resultFile.write(str(ncc) + "," + str(mf) + "," + str(fa) + ", " + result + "\n")
    plotResult(ncc, mf, fa)
    print("Test completed: " + str(ncc) + " " + str(mf) + " " + str(fa) + "\n")



resultFile = open("results.csv", "w+")
resultFile.write("NCC,MaxFeatures,FeatureAge,Result\n")


for i in range(0, ncc_it):
    ncc = ncc_base + 0.01 * i
    for y in range(0, mf_it):
        mf = mf_base + 2*y
        for x in range(0, fa_it):
            fa = fa_base + x
            print("ncc:" + str(ncc) + " mf:" + str(mf) + " fa: " + str(fa) + "\n")
            runTest(ncc, mf, fa)

resultFile.close()