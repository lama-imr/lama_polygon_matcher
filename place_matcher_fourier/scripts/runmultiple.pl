#!/usr/bin/env perl

use POSIX ":sys_wait_h";

my $max = 2;
my @p = ();
foreach my $a (1..$max) {
	$p[$a-1] = 0;
}

for(my $samples = 100; $samples<600;$samples+=50) {
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
		hard($samples);
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
	my ($samples) = @_;
#my $cmd="./runImageDataset-integralInvariant.pl dataset/MPEG7_CE-Shape-1_PartB-renamed-polygons/small/ $samples ee.$samples";
    my $cmd="./scripts/runImageDataset-integralInvariant.pl small/ $samples ee.$samples";
	print STDERR "Child runs '$cmd'\n";
	system $cmd;
}

