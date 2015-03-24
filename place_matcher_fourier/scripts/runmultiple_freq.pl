#!/usr/bin/env perl

use POSIX ":sys_wait_h";

my $max = 2;
my @p = ();
foreach my $a (1..$max) {
	$p[$a-1] = 0;
}

for(my $fftsize = 10; $fftsize < 100; $fftsize += 1) {
	my $freeidx = -1;
	do {
		foreach my $i (0..$max-1) {
			my $k = waitpid($p[$i],WNOHANG);
			if ($k == -1 and $freeidx == -1) {
				$freeidx = $i;
				print STDERR "Free is $a[$i]\n";
			}
		}
		if ($freeidx == -1) {
			sleep(1);
		}
	} while ($freeidx == -1);
	print STDERR "Main: free is $freeidx";
	$p[$freeidx] = fork();
	if ($p[$freeidx] == 0) {
		#child
		print STDERR "Child created: a=$a\n";
		hard($fftsize);
		print STDERR "Child exits a=$a\n";
		exit;
	} else {
		print STDERR "Parent: child $p[$freeidx] created\n";
	}
}

print STDERR "Waiting for all child processes:\n";
$k = waitpid(-1,0);
print STDERR "ok\n\n";


sub hard {
	my ($fftsize) = @_;
    my $cmd="./bin/tester 2 four.filelist $fftsize four.$fftsize";
	print STDERR "Child runs '$cmd'\n";
	system $cmd;
}

