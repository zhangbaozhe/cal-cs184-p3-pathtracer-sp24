import subprocess

def run1():
  for m in [2, 3, 4, 5]:
    subprocess.run(
      [
        "./build/pathtracer", 
        "-t", "12",
        "-s", "1024",
        "-l", "16",
        "-o", "0", 
        "-m", str(m),
        "-r", "480", "360", 
        "./dae/sky/CBbunny.dae", 
        "-f",
        "./docs/images/part4_CBbunny_s1024_l16_o0_m" + str(m) + ".png",
      ]
    )

def run2():
  for m in [0, 1, 2, 3, 4, 5]:
    subprocess.run(
      [
        "./build/pathtracer", 
        "-t", "12",
        "-s", "1024",
        "-l", "16",
        "-o", "1", 
        "-m", str(m),
        "-r", "480", "360", 
        "./dae/sky/CBbunny.dae", 
        "-f",
        "./docs/images/part4_CBbunny_s1024_l16_o1_m" + str(m) + ".png",
      ]
    )

def run3():
  for m in [0, 1, 2, 3, 4, 100]:
    subprocess.run(
      [
        "./build/pathtracer", 
        "-t", "12",
        "-s", "1024",
        "-l", "16",
        "-o", "1", 
        "-m", str(m),
        "-r", "480", "360", 
        "./dae/sky/CBbunny.dae", 
        "-f",
        "./docs/images/part4_CBbunny_s1024_l16_o1_m" + str(m) + "_russian.png",
      ]
    )

def run4():
  for s in [1, 2, 4, 8, 16, 64, 1024]:
    for m in [1, 2, 3, 4]:
      subprocess.run(
        [
          "./build/pathtracer", 
          "-t", "12",
          "-s", str(s),
          "-l", "4",
          "-o", "1", 
          "-m", str(m),
          "-r", "480", "360", 
          "./dae/sky/CBbunny.dae", 
          "-f",
          "./docs/images/part4_CBbunny_s{}_l4_o1_m{}_final.png".format(s, m),
        ]
      )

# run1()
# run2()

run3()
run4()